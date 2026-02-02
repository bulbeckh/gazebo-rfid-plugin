
#include "rfidmanager.h"

#include "components/rfidtagcomponent.h"
#include <gz/sim/components/Factory.hh>

#include <sdf/Element.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity_factory_v.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/plugin/Register.hh>

#include <vector>
#include <algorithm>

GZ_ADD_PLUGIN(RFIDManagerPlugin,
		gz::sim::System,
		RFIDManagerPlugin::ISystemConfigure,
		RFIDManagerPlugin::ISystemPreUpdate
)

GZ_SIM_REGISTER_COMPONENT("RFIDTag", RFIDTag)

RFIDManagerPlugin::RFIDManagerPlugin(void)
{

}

void RFIDManagerPlugin::Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr)
{
	// Store the ECM reference for internal use later
	ecm_internal = &_ecm;

	// Advertise tag creation service
	if (!this->node.Advertise<class RFIDManagerPlugin, gz::custom_msgs::RFIDTagList, gz::msgs::Boolean>(this->tagCreationServiceName,
				&RFIDManagerPlugin::tagCreateCallback,
				this)) {
		gzwarn << "Failed to create RFID tag creation service\n";
		return;
	} else {
		gzmsg << "Created RFID tag creation service\n";
	}

	// Advertise tag removal service
	if (!this->node.Advertise<class RFIDManagerPlugin, gz::custom_msgs::RFIDTagList, gz::msgs::Boolean>(this->tagRemovalServiceName,
				&RFIDManagerPlugin::tagRemovalCallback,
				this)) {
		gzwarn << "Failed to create RFID tag removal service\n";
		return;
	} else {
		gzmsg << "Created RFID tag removal service\n";
	}

	// Advertise all tag removal service
	if (!this->node.Advertise<class RFIDManagerPlugin, gz::msgs::Boolean>(this->tagAllRemovalServiceName,
				&RFIDManagerPlugin::tagAllRemovalCallback,
				this)) {
		gzwarn << "Failed to create RFID all tag removal service\n";
		return;
	} else {
		gzmsg << "Created RFID all tag removal service\n";
	}

	// Load the RFID Tag SDF Model
	sdf::Root root;
	sdf::Errors errors = root.Load("rfid-tag/model.sdf");

	if (!errors.empty()) {
		gzwarn << "Could not load rfid tag sdf\n";
	} else {
		// Convert to an SDF element and retrieve the model string
		sdf::ElementPtr tagModelSDFPtr = root.ToElement();

		tag_model_string = tagModelSDFPtr->ToString("");
	}

	return;
}

void RFIDManagerPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
					gz::sim::EntityComponentManager &_ecm)
{
	// On first run, we should find all loaded RFID tags and register the entities into the tag manager list
	if (!rfidManagerInitialised) {

		// Spawn tag
		gzmsg << "Attempting spawn of tag\n";

		// TODO Register all the tags that we found in the SDF

		rfidManagerInitialised = true;
	}

	// Create components for each tag not yet created
	while (!tag_component_list.empty()) {

		TagEntry te = tag_component_list.back();

		gz::sim::Entity tag_entity{gz::sim::kNullEntity};

		_ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
				[&](const gz::sim::Entity &_entity,
					const gz::sim::components::Model *,
					const gz::sim::components::Name *_name) -> bool
				{
					// Check if this entity matches our tag name
					if (_name->Data() == "rfid-tag-" + std::to_string(te.index)) {
        				tag_entity = _entity;
        				return false;
      				}
      				return true;
    			});

		if (tag_entity == gz::sim::kNullEntity) {
			// NOTE If we have not yet loaded an entity (and hence unable to create a component), the most straightforward
			// way to proceed is to ignore the remaining items and leave them for the next PreUpdate
			gzwarn << "[RFIDManager] Unable to find entity. Skipping creation of remaining components during this PreUpdate.";
			break;
		} else {
			// Create component for this tag
			auto* componentptr = _ecm.CreateComponent(tag_entity,
					RFIDTag({te.uid, te.data}));

			// If we successfully created, then remove from list
			if (componentptr != nullptr) {
				// TODO We sometimes run into an issue where PreUpdate runs before the entity is finished being created in the ECM so here we
				// attempt multiple times to create the component and only remove once done. We should add a counter to check that we haven't
				// attempted more than n times.
				tag_component_list.pop_back();

				gzwarn << "[RFIDManager] Created component for RFID Tag(" << tag_entity << ", " << te.uid << ", " << te.data << ")\n";
			}
		}
	}

	return;
}

bool RFIDManagerPlugin::tagCreateCallback(const gz::custom_msgs::RFIDTagList& req, gz::msgs::Boolean& reply)
{
	gzmsg << "Tag create callback triggered\n";

	gz::msgs::EntityFactory_V ef_v;
	bool result;
	gz::msgs::Boolean response;

	// This will be marked false if we fail to create any of the tags
	bool tag_create_success = true;

	// Iterate over all requested tags
	for (uint32_t i=0;i<req.tags_size();i++) {

		auto tag = req.tags(i);

		// Check that at minimum, our requested tag has both pose and a UID
		if (!tag.has_pose() || tag.uid() == "") {
			gzwarn << "[TagCreateCallback] Requested creation of tag but did not provide UID or pose. Skipping.\n";
			continue;
		}

		// Check that this tag UID has not already been requested
		if (std::find(uids.begin(), uids.end(), tag.uid()) != uids.end()) {
			gzwarn << "[TagCreateCallback] Requested creation of tag but UID is already used. Skipping.\n";
			continue;
		}

		gz::msgs::EntityFactory* ef = ef_v.add_data();

		// TODO Check that we actually retrieved the tag_model_string correctly
		ef->set_sdf(tag_model_string);

		ef->set_allow_renaming(true);

		std::string tag_name = "rfid-tag-" + std::to_string(tagIndex);
		ef->set_name(tag_name);

		ef->mutable_pose()->CopyFrom(tag.pose());

		// Create entry in our component list (to be added during PreUpdate)
		TagEntry tag_entry;
		tag_entry.index = tagIndex;
		tag_entry.uid = tag.uid();
		tag_entry.data = tag.data();

		tag_entry.pose = gz::msgs::Convert(tag.pose());

		tag_component_list.push_back(tag_entry);

		// Add UID to uid array
		uids.insert(tag.uid());

		gzwarn << "[TagCreateCallback] Creating tag: Tag(" << tag_entry.index << ", " << tag_entry.uid << ", " << tag_entry.data << ", " << tag_entry.pose << ")\n";

		// Increment tag index
		tagIndex += 1;
	}

	// Call entity creation service
	bool executed = node.Request("/world/rfid-test-world/create_multiple",
			ef_v,
			1000,
			response,
			result);

	if (executed && result) {
		gzmsg << "Entity created\n";
	} else {
		gzmsg << "Entity creation failed\n";
	}

	// Update the reply packet if we successully created all tags without error
	reply.set_data(executed && result && tag_create_success ? true : false);

	return true;
}

bool RFIDManagerPlugin::tagRemovalCallback(const gz::custom_msgs::RFIDTagList& req, gz::msgs::Boolean& reply)
{
	gzwarn << "Tag remove callback triggered\n";

	std::vector<std::string> remove_list;

	// TODO Check that request is correct format (i.e. has uid)

	// Iterate over all requested tags
	for (uint32_t i=0;i<req.tags_size();i++) {

		auto tag = req.tags(i);

		remove_list.push_back(tag.uid());

		gzwarn << tag.uid() << "\n";
	}

	// Iterate through models in simulation
	auto tag_entities = ecm_internal->EntitiesByComponents(gz::sim::components::Model());

	/* NOTE Alternative way to do the below is using the ecm Each function
	ecm_internal->Each<RFIDTag>(
			[&](const gz::sim::Entity &_entity, const RFIDTag *_tag) -> bool {
				const auto &tag = _tag->Data();
				gzwarn << "Found component/entity tag: " << tag.uid << "\n";
				return true;
				});
	*/

	gzwarn << "[TagRemove] Found " << tag_entities.size() << " entities with tag components\n";
	gzwarn << "[TagRemove] remove_list has  " << remove_list.size() << " entries\n";
	gzwarn << "[RFIDManager] uids set has " << uids.size() << " entries, and stack has " << tag_component_list.size() << " components waiting to be created\n";

	for (gz::sim::Entity t : tag_entities ) {
		//auto model = gz::sim::Model(t);
		//std::string model_name = model.Name(*ecm_internal);
		//gzwarn << "Model Name: " << model_name << "\n";

		// Retrieve RFID tag component
		auto* tag_component = ecm_internal->Component<RFIDTag>(t);

		if (tag_component) { 
			const auto& tag_component_data = tag_component->Data();

			// If we find this tag in the list
			if (std::find(remove_list.begin(), remove_list.end(), tag_component_data.uid) != remove_list.end()) {

				// Tag found, request removal of this entity
				gz::msgs::Entity entity_msg;
				bool result;
				gz::msgs::Boolean response;

				entity_msg.set_id(t);

				bool executed = node.Request("/world/rfid-test-world/remove",
						entity_msg,
						1000,
						response,
						result);

				// Remove from UID list
				if (result && executed) {
					uids.erase(tag_component_data.uid);
				}
			}
		}
		
		// Check if name starts with 'rfid-tag-'
		/*
		if (model_name.compare(0, tag_prefix.length(), tag_prefix) == 0) {

			gzwarn << "Found matching tag for removal\n";

			if (std::find(remove_list.begin(), remove_list.end(), model_name.substr(9)) != remove_list.end()) {
				// Tag found, request removal of this entity
				gz::msgs::Entity entity_msg;
				bool result;
				gz::msgs::Boolean response;

				entity_msg.set_id(model.Entity());

				bool executed = node.Request("/world/rfid-test-world/remove",
						entity_msg,
						1000,
						response,
						result);

				// Remove from UID list
				if (result && executed) {

				}

			}
		}
		*/
	}

	// TODO
	// For each of the tags in req, get the simulation name of the tag, get the corresponding entity and then call the
	// remove service for that entity. Also remove the tag from the database

	return true;
}

bool RFIDManagerPlugin::tagAllRemovalCallback(gz::msgs::Boolean& reply)
{
	gz::msgs::Entity entity_msg;
	bool result;
	gz::msgs::Boolean response;

	// Remove all RFIDTag entities
	ecm_internal->Each<RFIDTag>(
			[&](const gz::sim::Entity &_entity, const RFIDTag *_tag) -> bool {

				entity_msg.set_id(_entity);

				bool executed = node.Request("/world/rfid-test-world/remove",
						entity_msg,
						1000,
						response,
						result);

				gzwarn << "Removing tag " << _tag->Data().uid << "\n";
				return true;
			});

	// Clear the UID list
	uids.clear();

	/*
	for (gz::sim::Entity t : tag_entities ) {
		auto model = gz::sim::Model(t);
		
		// Check if name starts with 'rfid-tag-'
		if (model.Name(*ecm_internal).compare(0, tag_prefix.length(), tag_prefix) == 0) {

			// Update the entity for this model and call the removal service
			entity_msg.set_id(model.Entity());

			bool executed = node.Request("/world/rfid-test-world/remove",
					entity_msg,
					1000,
					response,
					result);
		}
	}
	*/
	
	return false;
}
