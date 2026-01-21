
#include "rfidscanner.h"

#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>

#include <gz/math/Pose3.hh>

#include <gz/sim/components/Model.hh>

#include <gz/plugin/Register.hh>

#include <string>
#include <cmath>

GZ_ADD_PLUGIN(RFIDScannerPlugin,
		gz::sim::System,
		RFIDScannerPlugin::ISystemConfigure,
		RFIDScannerPlugin::ISystemPreUpdate
)

RFIDScannerPlugin::RFIDScannerPlugin(void) :
	gen(rd()),
	distribution(0,1)
{
}

double RFIDScannerPlugin::sigmoid(double x)
{
	return 1.0 / (1.0 + std::exp(-x));
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

	// Retrieve scanner orientation
	gz::math::Quaterniond& scanner_orientation = sp->Rot();

	/* Here is where we iterate over all the tags in the simulation and check to see whether they meet the criteria
	 * to be scanned.
	 *
	 * Our RFIDScanResponse message has the following format:
	 *
	 * TODO
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

			// TODO Need to make this far more advanced (as per RFID model)
			/* Here, we determine if the tag should be scanned as per our RFID model. We have a number of
			 * parameters that are retrieved from the SDF during the Configure method.
			 *
			 * In particular, we are interested in the relative orientation between the tag and the scanner,
			 * as well as the linear distance between the two.
			 * 
			 * See the blog post at TODO for the full derivation of the equations but we need to calculate
			 *
			 * TODO Add in the antenna (directional) gain term (Gt)
			 * TODO 
			 *
			 */
			if (wp && sp) {
				// Calculate linear distance
				double tag_scanner_linear_distance = (wp->CoordPositionSub(*sp)).Length();

				// Calculate dot product between scanner and tag
				gz::math::Quaterniond& tag_orientation = wp->Rot();
				double cos_theta = tag_orientation.Dot(scanner_orientation);

				// Calculate tranmission power
				
				// TODO This is wrong - the max should be applied before(?) the cosine
				double loss_polarization = std::min(
						polarization_max_loss,
						-20*std::log10(
							std::max(polarization_minimum_angle, std::abs(cos_theta))
						)
					);

				// TODO Replace with the check for LOS / NLOS between reader and tag
				double loss_path;
				if (true) {
					loss_path = 10*path_loss_los_gain*std::log10(tag_scanner_linear_distance / path_loss_scaling);
				} else {
					loss_path = 10*path_loss_nlos_gain*std::log10(tag_scanner_linear_distance / path_loss_scaling);
				}

				double transmission_power = std::min(100.0,
						std::max(-20.0, antenna_gain + antenna_directional_gain + tag_directional_gain - loss_polarization - loss_path)
						);

				double received_power = std::min(100.0,
						std::max(-20.0, antenna_gain + antenna_directional_gain + tag_directional_gain - 2*(loss_polarization + loss_path)));

				// Calculate read probabilities
				
				double p_read_tx = sigmoid((transmission_power - tx_threshold_power) / tx_read_scaling);
				double p_read_rx = sigmoid((received_power - rx_threshold_power) / rx_read_scaling);

				double p_read = p_read_tx*p_read_rx;

				gzwarn << model.Name(*ecm_internal) << " " << *wp << " , power: " << transmission_power << ", p(read): " << p_read << "\n";

				if ( distribution(gen) < p_read ) {
					// Successful read - add tag to list of found tags

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

