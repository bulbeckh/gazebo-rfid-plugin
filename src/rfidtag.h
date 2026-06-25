
#pragma once

#include <gz/sim/System.hh>

class RFIDTag :
	public gz::sim::System,
	public gz::sim::ISystemConfigure
{

	public: RFIDTag();

	public: ~RFIDTag() override = default;

	public: void Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr) final;

};

