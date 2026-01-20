
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
	// Fail if we have not yet initialised the scanner
	if (!scanner_initialised) {
		gzwarn << "Scan requested but scanner not yet initialised\n";
		return false;

	}

	gzwarn << "Scan requested\n";

	// TODO We should change the ecm_internal object to a const ref/pointer to ensure no modification of the ECM
	// Iterate over all models
	auto tag_entities = ecm_internal->EntitiesByComponents(
			gz::sim::components::Model());

	gzwarn << "In loop " << tag_entities.size() << " models found\n";
	gzwarn << "Time: " << simulation_time_sec << "s " << simulation_time_nsec << "ms\n";

	// Add scan timestamp
	auto scan_time = _reply.mutable_time();
	scan_time->set_sec(simulation_time_sec);
	scan_time->set_nsec(simulation_time_nsec);

	// Retrieve current pose of the scanner
	auto sp = scanner_link.WorldPose(*ecm_internal);

	/* Here is where we iterate over all the tags in the simulation and check to see whether they meet the criteria
	 * to be scanned.
	 *
	 * Our RFIDScanResponse message has the following format:
	 *
	 *
	 *
	 *
	 */
	for (gz::sim::Entity t : tag_entities ) {
		auto model = gz::sim::Model(t);
		
		// Check if name starts with 'rfid-tag-'
		if (model.Name(*ecm_internal).compare(0, tag_prefix.length(), tag_prefix) == 0) {

			gz::sim::Entity model_link_entity = model.CanonicalLink(*ecm_internal);

			gz::sim::Link link = gz::sim::Link(model_link_entity);

			// Retrieve pose of tag
			auto wp = link.WorldPose(*ecm_internal);

			if (wp && sp) {
				//gz::math::Pose3d model_pose = *wp;
				gzwarn << model.Name(*ecm_internal) << "tag pose(" << *wp << ") scanner pose(" << *sp << ")\n";

				// Calculate linear distance
				//gzwarn << "Linear distance: " << (wp->CoordPositionSub(*sp)).Length() << "\n";
				
				double tag_scanner_linear_distance = (wp->CoordPositionSub(*sp)).Length();

				// TODO Need to make this far more advanced (as per RFID model)
				if (tag_scanner_linear_distance < 100.0) {
					// Add tag to list of found tags

					auto* scanmessage = _reply.add_scan();

					scanmessage->set_tag_data(model.Name(*ecm_internal));

					// TODO Add these fields
					scanmessage->set_tag_id(5);
					scanmessage->set_rssi(100.0);
				}
			}
		}
	}
	
	return true;
}

void RFIDScannerPlugin::Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr)
{
	// Store the entity for later
	scanner_entity = _entity;

	// Store the ECM reference for internal use later
	ecm_internal = &_ecm;

	// Expose the scan service
	if (!this->node.Advertise<class RFIDScannerPlugin, gz::custom_msgs::RFIDScanResponse>(this->scan_service_name,
				&RFIDScannerPlugin::scanRequestCallback,
				this)) {
		gzwarn << "Failed to create RFID scan service\n";
		return;
	} else {
		gzmsg << "Created RFID scan service\n";
	}

	return;
}

void RFIDScannerPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm)
{
	// Get the canonical link of the scanner so that we can retrieve the pose later
	if (!scanner_initialised) {
		auto scanner_model = gz::sim::Model(scanner_entity);

		scanner_link = gz::sim::Link(scanner_model.CanonicalLink(_ecm));

		scanner_initialised = true;
	}

	// TODO Not thread-safe
	// Track the most recent simulation time so that we can use it during the callback
	
	simulation_time_sec = (int32_t)(std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime).count());

	simulation_time_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count(); /* - simulation_time_sec*1e3; */

	return;
}

