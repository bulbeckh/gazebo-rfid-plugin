#pragma once

#include <gz/sim/System.hh>

#include <memory>

// Forward decl
class RFIDScannerPrivate;

class RFIDScanner :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{

	public: RFIDScanner();

	// TODO
	public: ~RFIDScanner() override = default;

	public: void Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr) final;

	public: void PreUpdate(const gz::sim::UpdateInfo &_info,
					gz::sim::EntityComponentManager &_ecm) final;

	private: std::unique_ptr<RFIDScannerPrivate> dataPtr;
};


