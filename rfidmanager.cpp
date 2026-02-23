
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

		tag_sdf_loaded = true;
	}

	return;
}

void RFIDManagerPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
					gz::sim::EntityComponentManager &_ecm)
{
	// TODO Add functionality to spawn tags from SDF file (rather than via service)

	// Create components for each tag not yet created
	while (!tag_component_list.empty()) {

		TagEntry te = tag_component_list.back();

		gz::sim::Entity tag_entity{gz::sim::kNullEntity};

		// Iterate over all model model entities to find our desired tag
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
			gzwarn << "Unable to find entity. Skipping creation of remaining components during this PreUpdate.\n";
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

				gzmsg << "Created tag component: (entity:" << tag_entity << ", uid: " << te.uid << ", data: " << te.data << ")\n";
			}
		}
	}

	return;
}

bool RFIDManagerPlugin::tagCreateCallback(const gz::custom_msgs::RFIDTagList& req, gz::msgs::Boolean& reply)
{

	// If we have not sucessfully loaded the tag SDF, then we won't be able to spawn the tag
	if (!tag_sdf_loaded) {
		gzwarn << "Unable to load tag SDF. Will not spawn tags\n";
	}

	gz::msgs::EntityFactory_V ef_v;
	bool result;
	gz::msgs::Boolean response;

	// This will be marked false if we fail to create any of the tags. We will attempt to create as many tags as possible via the
	// entity creation service but our response to the original tag creation request will be 'false' if we are unable to create at
	// least 1 tag.
	bool tag_create_success = true;

	// Iterate over all requested tags
	for (uint32_t i=0;i<req.tags_size();i++) {

		auto tag = req.tags(i);

		// Check that at minimum, our requested tag has both pose and a UID
		if (!tag.has_pose() || tag.uid() == "") {
			gzwarn << "Requested creation of tag but did not provide UID or pose. Skipping.\n";
			tag_create_success = false;
			continue;
		}

		// Check that this tag UID has not already been requested
		if (std::find(uids.begin(), uids.end(), tag.uid()) != uids.end()) {
			gzwarn << "Requested creation of tag but UID is already used. Skipping.\n";
			tag_create_success = false;
			continue;
		}

		// Add tag information to entity factory message
		gz::msgs::EntityFactory* ef = ef_v.add_data();

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

		gzmsg << "Created tag entity: (tag_name: " << tag_name << ", pose: " << tag_entry.pose << ")\n";

		// Increment tag index
		tagIndex += 1;
	}

	// Call entity creation service
	bool executed = node.Request("/world/rfid-test-world/create_multiple",
			ef_v,
			1000,
			response,
			result);

	if (!executed) gzwarn << "Failed to call entity creation service\n";

	// Update the reply packet if we successully created all tags without error
	reply.set_data(executed && result && tag_create_success ? true : false);

	return true;
}

bool RFIDManagerPlugin::tagRemovalCallback(const gz::custom_msgs::RFIDTagList& req, gz::msgs::Boolean& reply)
{
	bool tag_remove_success = true;

	std::vector<std::string> remove_list;

	// TODO Check that request is correct format (i.e. has uid)

	// Get a list of UIDs of tags to be removed
	for (uint32_t i=0;i<req.tags_size();i++) {
		auto tag = req.tags(i);

		// If we were not provided a uid, then skip the tag
		if (tag.uid() == "") {
			gzwarn << "Requested to remove a tag but did not provide a uid. Skipping tag\n";
		}

		remove_list.push_back(tag.uid());
	}

	// Iterate through all tags and if they appear in the list of tags to be removed, then request removal of entity
	ecm_internal->Each<RFIDTag>(
			[&](const gz::sim::Entity &_entity, const RFIDTag *_tag) -> bool {

				const auto &tag = _tag->Data();

				auto remove_tag_idx = std::find(remove_list.begin(), remove_list.end(), tag.uid);

				if (remove_tag_idx != remove_list.end()) {
					// Tag found - request removal of entity and then remove from remove_list vector

					// TODO
					// Tag found, request removal of this entity
					gz::msgs::Entity entity_msg;
					bool result;
					gz::msgs::Boolean response;

					entity_msg.set_id(_entity);

					bool executed = node.Request("/world/rfid-test-world/remove",
							entity_msg,
							1000,
							response,
							result);

					// Remove from UID list
					if (result && executed) {
						uids.erase(tag.uid);
						gzmsg << "Removed tag: (uid: " << tag.uid << ")\n";
					} else {
						tag_remove_success = false;
						gzwarn << "Failed to remove tag: " << tag.uid << "\n";
					}

					// Remove from remove_list
					remove_list.erase(remove_tag_idx);
				}

				gzwarn << "Found component/entity tag: " << tag.uid << "\n";
				return true;
			});

	// Confirm that we have successfully removed all tags
	if (remove_list.size() != 0) {
		tag_remove_success = false;

		gzwarn << "Requested removal of tags but was unable to find the following tags: ";
		for (auto t : remove_list) {
			gzwarn << t << " ";
		}
		gzwarn << "\n";
	}

	// Update reply
	reply.set_data(tag_remove_success);

	return true;
}

bool RFIDManagerPlugin::tagAllRemovalCallback(gz::msgs::Boolean& reply)
{
	gz::msgs::Entity entity_msg;
	bool result;
	gz::msgs::Boolean response;

	bool tag_remove_success = true;

	// Remove all RFIDTag entities
	ecm_internal->Each<RFIDTag>(
			[&](const gz::sim::Entity &_entity, const RFIDTag *_tag) -> bool {

				entity_msg.set_id(_entity);

				bool executed = node.Request("/world/rfid-test-world/remove",
						entity_msg,
						1000,
						response,
						result);

				if (!executed) {
					tag_remove_success = false;
					gzwarn << "Failed to remove tag\n";
				} else {
					gzmsg << "Removed tag: (uid: " << _tag->Data().uid << ")\n";
				}

				return true;
			});

	// Clear the full UID list
	uids.clear();

	// Mark reply as success or fail
	reply.set_data(tag_remove_success ? true : false);

	return true;
}
