
#include "rfidmanager.h"

#include <sdf/Element.hh>

#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity_factory_v.pb.h>

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(RFIDManagerPlugin,
		gz::sim::System,
		RFIDManagerPlugin::ISystemConfigure,
		RFIDManagerPlugin::ISystemPreUpdate
)

RFIDManagerPlugin::RFIDManagerPlugin(void)
{

}

void RFIDManagerPlugin::Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr)
{
	// Advertise the two tag creation and removal services
	if (!this->node.Advertise<class RFIDManagerPlugin, gz::custom_msgs::RFIDCreateRequest, gz::custom_msgs::RFIDCreateResponse>(this->tagCreationServiceName,
				&RFIDManagerPlugin::tagCreateCallback,
				this)) {
		gzwarn << "Failed to create RFID tag creation service\n";
		return;
	} else {
		gzmsg << "Created RFID tag creation service\n";
	}

	if (!this->node.Advertise<class RFIDManagerPlugin, gz::msgs::StringMsg, gz::msgs::StringMsg>(this->tagRemovalServiceName,
				&RFIDManagerPlugin::tagRemovalCallback,
				this)) {
		gzwarn << "Failed to create RFID tag removal service\n";
		return;
	} else {
		gzmsg << "Created RFID tag removal service\n";
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

	return;
}

bool RFIDManagerPlugin::tagCreateCallback(const gz::custom_msgs::RFIDCreateRequest& req, gz::custom_msgs::RFIDCreateResponse& reply)
{
	gzmsg << "Tag create callback triggered\n";

	gz::msgs::EntityFactory_V ef_v;
	bool result;
	gz::msgs::Boolean response;

	// Iterate over all requested tags
	for (uint32_t i=0;i<req.tags_size();i++) {

		gz::msgs::EntityFactory* ef = ef_v.add_data();

		auto tag = req.tags(i);

		// TODO Check that we actually retrieved the tag_model_string correctly
		ef->set_sdf(tag_model_string);

		ef->set_allow_renaming(true);

		std::string tag_name = "rfid-tag-" + std::to_string(tagIndex);
		ef->set_name(tag_name);

		ef->mutable_pose()->CopyFrom(tag.pose());

		// TODO Somehow add the category information into the model (maybe via model name)
		
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

	// TODO Update the reply packet with this created tag

	return true;
}

bool RFIDManagerPlugin::tagRemovalCallback(const gz::msgs::StringMsg& req, gz::msgs::StringMsg& reply)
{
	// TODO
	return false;
}

uint32_t RFIDManagerPlugin::addTag(gz::math::Pose3d pose, std::string tag_id)
{
	// TODO
	return 0;
}

bool RFIDManagerPlugin::removeTag(std::string tag_id)
{
	// TODO
	return false;
}

bool RFIDManagerPlugin::removeTag(uint32_t tag_id)
{
	// TODO
	return false;
}

bool RFIDManagerPlugin::removeAllTags(void)
{
	// TODO
	return false;
}


