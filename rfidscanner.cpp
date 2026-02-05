
#include "rfidscanner.h"
#include "components/rfidtagcomponent.h"

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



	// NOTE New iteration
	ecm_internal->Each<RFIDTag>(
			[&](const gz::sim::Entity &_entity, const RFIDTag *_tag) -> bool {
				const auto &tag = _tag->Data();

				gzwarn << "Found component/entity tag: " << tag.uid << "\n";

				return true;
			});

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
				// To estimate rssi (and read probability), we need both the scanner pose and the tag pose
				//
				// From these, we calculate:
				// 1. Polarization (difference in angle between tag orientation and scanner orientation
				// 2. Linear distance (scalar difference in position between scanner and tag)
				// 3. Antenna gain (difference in angle between antenna boresight vector and scanner to tag vector)

				// Vector from scanner to tag in world frame
				gz::math::Vector3d scanner_pose_vector = wp->CoordPositionSub(*sp);

				double tag_scanner_linear_distance = scanner_pose_vector.Length();


				// Calculate dot product between scanner orientation and tag orientation (for polarization calculation)
				
				// TODO NOTE This vector represents that orientation of the tag polarization in its default state. Currently,
				// this is pointing towards the tag x-axis but this needs to be configured.
				gz::math::Vector3d tag_local_ref(1,0,0);

				// TODO Make sure this reconciles with our SDF model of antenna
				// We use (1,0,0) because in the default orientation (no rotation) the scanner boresight is pointing in y-axis
				gz::math::Vector3d antenna_local_ref(1,0,0);

				gz::math::Quaterniond& tag_orientation = wp->Rot();

				double cos_theta = (tag_orientation.RotateVector(tag_local_ref)).Dot(scanner_orientation.RotateVector(antenna_local_ref));

				// Calculate dot product between scanner to tag vector and scanner orientation (for antenna gain calculation)

				gz::math::Vector3d scanner_direction = scanner_orientation.RotateVector(antenna_local_ref);

				// Relative angle between antenna boresight and direction to tag
				double antenna_gain_angle = std::acos(scanner_direction.Dot(scanner_pose_vector.Normalized()));


				// Calculate tranmission power
				//
				// In default configuration, equation is: antenna_gain = 6 - min(25, 6*(\theta^{2}))
				
				double antenna_gain = antenna_gain_peak - std::min(antenna_gain_max_loss, antenna_gain_loss_scaling*std::pow(antenna_gain_angle,2));

				// Calculate polarization loss
				//
				// In default configuration, equation is: polarization_loss = min(25, log10(cos(\theta)))
				
				double loss_polarization = std::min(polarization_max_loss, -20*std::log10(std::abs(cos_theta)));

				// TODO Add LOS / NLOS factor between reader and tag
				// Calculate path loss
				//
				// In default configuration, equation is: path_loss = 31 + 10*2.2*los10(max(0.2, distance))

				double loss_path = path_loss_base_loss + 10*path_loss_los_gain*std::log10(std::max(path_loss_min_distance, tag_scanner_linear_distance));

				// Calculate tranmission and received power
				double transmission_power = antenna_power + antenna_gain + tag_directional_gain - loss_polarization - loss_path;
				double received_power = antenna_power + antenna_gain + tag_directional_gain - 2*(loss_polarization + loss_path);

				// Calculate read probabilities
				double p_read_tx = sigmoid((transmission_power - tx_threshold_power) / tx_read_scaling);
				double p_read_rx = sigmoid((received_power - rx_threshold_power) / rx_read_scaling);
				double p_read = p_read_tx*p_read_rx;

				// DEBUG
				if (true) {
					gzwarn << model.Name(*ecm_internal) << "\n";
					gzwarn << "    Scanner Pose3d     (m,rad): " << *sp << "\n";
					gzwarn << "    Tag Pose3d         (m,rad): " << *wp << "\n";
					gzwarn << "    Scanner-Tag Pose3d (m,rad): " << scanner_pose_vector << "\n";
					gzwarn << "    Scanner Quaternion        : " << sp->Rot() << "\n";
					gzwarn << "    Scanner Eul               : " << sp->Rot().Euler() << "\n";
					gzwarn << "    Tag Quaternion            : " << wp->Rot() << "\n";
					gzwarn << "    Tag Eul                   : " << wp->Rot().Euler() << "\n";
					gzwarn << "    Scanner-Tag Vec        (m): " << scanner_pose_vector << "\n";
					gzwarn << "    Polarization angle   (rad): " << std::acos(cos_theta) << "\n";
					gzwarn << "    Scanner direction vec  (m): " << scanner_direction << "\n";
					gzwarn << "    Antenna gain angle (cos(rad)): " << antenna_gain_angle << "\n";
					gzwarn << "    Linear distance   : " << tag_scanner_linear_distance << "\n";
					gzwarn << " Transmission Stats\n";
					gzwarn << "	   antenna gain: " << antenna_gain << "\n";
					gzwarn << "	   loss polarization: " << loss_polarization << "\n";
					gzwarn << "	   loss path: " << loss_path << "\n";
					gzwarn << "	   transmission power: " << transmission_power << "\n";
					gzwarn << "	   received power: " << received_power << "\n";
				}

				// Sample from distribution to determine if read was successful
				if ( distribution(gen) < p_read ) {

					// Successful read - add tag to list of found tags
					auto* scanmessage = _reply.add_scan();

					scanmessage->set_data(model.Name(*ecm_internal));

					// TODO Add these fields
					scanmessage->set_uid("5");
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

	// Retrieve configuration from the sdf
	
	if (_sdf->HasElement("antenna_power")) antenna_power = _sdf->Get<double>("antenna_power");

	if (_sdf->HasElement("path_loss_los_gain")) path_loss_los_gain = _sdf->Get<double>("path_loss_los_gain");
	if (_sdf->HasElement("path_loss_base_loss")) path_loss_base_loss = _sdf->Get<double>("path_loss_base_loss");
	if (_sdf->HasElement("path_loss_min_distance")) path_loss_min_distance = _sdf->Get<double>("path_loss_min_distance");

	if (_sdf->HasElement("polarization_max_loss")) polarization_max_loss = _sdf->Get<double>("polarization_max_loss");

	if (_sdf->HasElement("antenna_gain_peak")) antenna_gain_peak = _sdf->Get<double>("antenna_gain_peak");
	if (_sdf->HasElement("antenna_gain_max_loss")) antenna_gain_max_loss = _sdf->Get<double>("antenna_gain_max_loss");
	if (_sdf->HasElement("antenna_gain_loss_scaling")) antenna_gain_loss_scaling = _sdf->Get<double>("antenna_gain_loss_scaling");

	if (_sdf->HasElement("tag_directional_gain")) tag_directional_gain = _sdf->Get<double>("tag_directional_gain");

	if (_sdf->HasElement("tx_threshold_power")) tx_threshold_power = _sdf->Get<double>("tx_threshold_power");
	if (_sdf->HasElement("rx_threshold_power")) rx_threshold_power = _sdf->Get<double>("rx_threshold_power");

	if (_sdf->HasElement("tx_read_scaling")) tx_read_scaling = _sdf->Get<double>("tx_read_scaling");
	if (_sdf->HasElement("rx_read_scaling")) rx_read_scaling = _sdf->Get<double>("rx_read_scaling");

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

