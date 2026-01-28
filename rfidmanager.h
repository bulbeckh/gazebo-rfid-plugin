#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

#include <gz/custom_msgs/rfid_tag_list.pb.h>

#include <sdf/sdf.hh>

#include <map>

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

		std::string tag_model_string;

		/* @brief Whether we have initialised the tag manager */
		bool rfidManagerInitialised{false};

		/* @brief Index of the next created tag. Used to specify tag names */
		uint64_t tagIndex{0};

		// Callback methods
		
		/* @brief Callback method for tag creation requests */
		bool tagCreateCallback(const gz::custom_msgs::RFIDTagList& req, gz::custom_msgs::RFIDTagList& reply);

		// TODO Update to new message format
		/* @brief Callback method for tag removal requests */
		bool tagRemovalCallback(const gz::custom_msgs::RFIDTagList& req, gz::custom_msgs::RFIDTagList& reply);

		/* @brief Callback method for removing all tags in simulation */
		bool tagAllRemovalCallback(gz::custom_msgs::RFIDTagList& reply);

	private:
		/* @brief An object used to store tag entries in our database */
		struct TagEntry {
			std::string simname;
			std::string data;
		};

		/* @brief A mapping from a tag UID to either the underlying data or the simulation name or the tag */
		std::map<std::string, TagEntry> tagmap;

};

