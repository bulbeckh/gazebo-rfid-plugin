#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <gz/custom_msgs/rfid_tag_list.pb.h>

#include <sdf/sdf.hh>

#include <set>
#include <vector>

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
		std::string tagCreationServiceName = "/rfid_tag_create";

		/* @brief Tag remove service name */
		std::string tagRemovalServiceName = "/rfid_tag_remove";
		std::string tagAllRemovalServiceName = "/rfid_tag_remove_all";

		/* @brief String containing the full SDF code of the tag. Loaded during Configure */
		std::string tag_model_string;

		// TODO There is a better way to identify models (entities) as tags than check the name. Perhaps use components
		/* @brief The prefix used to identify a model as a tag model */
		std::string tag_prefix{"rfid-tag-"};

		/* @brief Whether we have initialised the tag manager */
		bool rfidManagerInitialised{false};

		/* @brief Index of the next created tag. Used to specify tag names */
		uint64_t tagIndex{0};

		// Callback methods
		
		/* @brief Callback method for tag creation requests */
		bool tagCreateCallback(const gz::custom_msgs::RFIDTagList& req, gz::msgs::Boolean& reply);

		// TODO Update to new message format
		/* @brief Callback method for tag removal requests */
		bool tagRemovalCallback(const gz::custom_msgs::RFIDTagList& req, gz::msgs::Boolean& reply);

		/* @brief Callback method for removing all tags in simulation */
		bool tagAllRemovalCallback(gz::msgs::Boolean& reply);

		/* @brief A pointer to the ECM. NOTE We can do this as long as we don't use it to modify the ecm (that should be done
		 * during PreUpdate/PostUpdate) */
		gz::sim::EntityComponentManager* ecm_internal{nullptr};

	private:
		/* @brief An object used to store tag entries in our database */
		struct TagEntry {
			uint64_t index;
			std::string uid;
			std::string data;
			gz::math::Pose3d pose;
		};

		/* @brief Store of the world name for the tag creation/removal calls */
		std::string world_name;

		/* @brief A list of tag components that have not yet been added to the simulation. Tags are added during the callback, but
		 * the component creation is done during PreUpdate */
		std::vector<TagEntry> tag_component_list;

		/* @brief A set containing the UIDs of tags. Primarily used to check if a tag UID has already been created */
		std::set<std::string> uids;

		/* @brief Flag to determine if we have correctly loaded SDF */
		bool tag_sdf_loaded{false};

};

