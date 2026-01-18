
#include "rfidscanner.h"

#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>

#include <gz/math/Pose3.hh>

#include <gz/sim/components/Model.hh>

#include <gz/plugin/Register.hh>

#include <string>

GZ_ADD_PLUGIN(RFIDScannerPlugin,
		gz::sim::System,
		RFIDScannerPlugin::ISystemConfigure,
		RFIDScannerPlugin::ISystemPreUpdate
)

RFIDScannerPlugin::RFIDScannerPlugin(void)
{
}

bool RFIDScannerPlugin::scanRequestCallback(gz::custom_msgs::RFIDScanResponse& _reply)
{
	// TODO Retrieve most recent scan (and the position the scan was conducted at)
	
	return true;
}

void RFIDScannerPlugin::Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr)
{
	// Store the entity for later
	scanner_entity = _entity;


	return;
}

void RFIDScannerPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm)
{
	if (!scanner_initialised) {
		// Get the canonical link of the scanner so that we can retrieve the pose later
		auto scanner_model = gz::sim::Model(scanner_entity);

		scanner_link = gz::sim::Link(scanner_model.CanonicalLink(_ecm));
	}

	// Run scan every n iterations
	if (scan_index % 1000 == 0 /* == scan_period */) {

		// Iterate over all models
		auto tag_entities = _ecm.EntitiesByComponents(
				gz::sim::components::Model());

		gzwarn << "In loop " << tag_entities.size() << " models found\n";

		// Retrieve current pose of the scanner
		auto sp = scanner_link.WorldPose(_ecm);

		for (gz::sim::Entity t : tag_entities ) {
			auto model = gz::sim::Model(t);
			
			// Check if name starts with 'rfid-tag-'
			if (model.Name(_ecm).compare(0, tag_prefix.length(), tag_prefix) == 0) {

				gz::sim::Entity model_link_entity = model.CanonicalLink(_ecm);

				gz::sim::Link link = gz::sim::Link(model_link_entity);

				// Retrieve pose of tag
				auto wp = link.WorldPose(_ecm);

				if (wp && sp) {
					//gz::math::Pose3d model_pose = *wp;
					gzwarn << model.Name(_ecm) << "tag pose(" << *wp << ") scanner pose(" << *sp << ")\n";

					// Calculate linear distance
					gzwarn << "Linear distance: " << (wp->CoordPositionSub(*sp)).Length() << "\n";
				}
			}
		}
	}

	scan_index++;

	return;
}

