
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>

#include <chrono>
#include <random>

#include <sdf/sdf.hh>

class RFIDTagPlugin :
	public gz::sim::System,
	public gz::sim::ISystemConfigure
{

	public:
		RFIDTagPlugin();

	public:
		void Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr) final;

	private:
		/* @brief Whether we have successfully added a component to the ECM for this tag */
		bool _component_added{false};

		/* @brief UID associated with this tag */
		std::string tag_uid;

		/* @brief Data stored on this tag. Limited to 512 bytes */
		std::string tag_data;


};

