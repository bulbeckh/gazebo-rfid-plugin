
#include "rfidscanner.h"


RFIDScannerPlugin::RFIDScannerPlugin(void)
{
}

bool RFIDScannerPlugin::scanRequestCallback(gz::custom_msgs::RFIDScanResponse& _reply)
{
	// TODO
	return true;
}

void RFIDScannerPlugin::Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr)
{
	return;
}

void RFIDScannerPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm)
{

	if (scan_index == scan_period) {


	}

	return;
}

