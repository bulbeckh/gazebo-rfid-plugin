#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

#include <gz/custom_msgs/rfid_create_request.pb.h>
#include <gz/custom_msgs/rfid_create_response.pb.h>

#include <sdf/sdf.hh>

/* RFID Manager class needs to do the following:
 *
 * - Create RFID tags, given a 3d location, by exposing a gazebo message node
 * - Remove RFID tags, given an identifier, by exposing a gazebo message node
 * - Create RFID tags, given a 3d location, through the SDF interface
 *
 * Creation of tags via the SDF interface
 *
 * We specify tags by including them as models */

class RFIDManagerPlugin :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{

	public:
		RFIDManagerPlugin(void);

	public:
		void Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr) final;

		void PreUpdate(const gz::sim::UpdateInfo &_info,
					gz::sim::EntityComponentManager &_ecm) final;

	private:

		gz::transport::Node node;

		/* @brief Tag create service name */
		std::string tagCreationServiceName = "/rfidtagcreate";

		/* @brief Tag remove service name */
		std::string tagRemovalServiceName = "/rfidtagremove";

		std::string tag_model_string;

		/* @brief Whether we have initialised the tag manager */
		bool rfidManagerInitialised{false};

		/* @brief Index of the next created tag. Used to specify tag names */
		uint64_t tagIndex{0};

		/* @brief Callback method for tag creation requests */
		bool tagCreateCallback(const gz::custom_msgs::RFIDCreateRequest& req, gz::custom_msgs::RFIDCreateResponse& reply);

		// TODO Update to new message format
		/* @brief Callback method for tag removal requests */
		bool tagRemovalCallback(const gz::msgs::StringMsg& req, gz::msgs::StringMsg& reply);

		/* @brief Add a tag to the world in the provided location
		 * @param pose The position of the tag. Orientation is ignored
		 * @param tag_id An string containing some sort of identification of the tag. Optional. */
		uint32_t addTag(gz::math::Pose3d pose, std::string tag_id);

		/* @brief Removes the specified tag from the world */
		bool removeTag(std::string tag_id);

		/* @brief Removes the specified tag from the world */
		bool removeTag(uint32_t tag_id);

		/* @brief Removes all tags from the world */
		bool removeAllTags(void);


};

