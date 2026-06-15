
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>

#include <chrono>
#include <random>

#include <sdf/sdf.hh>

/* This is a basic RFID Tag class, with the only functionality being to add a Component to the associated model */
class RFIDTagPlugin :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{

	public:
		RFIDTagPlugin();

	public:
		void Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr) final;

		void PreUpdate(const gz::sim::UpdateInfo &_info,
					gz::sim::EntityComponentManager &_ecm) final;

	private:
		/* @brief Whether we have added a component to the ECM for this tag */
		bool _component_added{false};

		std::string tag_uid;
		std::string tag_data;


};

