
#include "rfidscanner.h"

#include "../components/rfidtagcomponent.h"

#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>

#include <gz/math/Pose3.hh>

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Pose.hh>

#include <gz/plugin/Register.hh>

#include <string>
#include <cmath>

#include <chrono>
#include <random>
#include <deque>
#include <mutex>

#include <gz/transport/Node.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <gz/custom_msgs/rfid_scan_response.pb.h>

#include <sdf/sdf.hh>



double sigmoid(double x)
{
	return 1.0 / (1.0 + std::exp(-x));
}

struct PendingScan
{
	public: std::promise<bool> promise;
	public: gz::custom_msgs::RFIDScanResponse* _reply;
};

class RFIDScannerPrivate
{
	public: RFIDScannerPrivate();

	/* @brief Node for the scan service */
	public: gz::transport::Node node;

	/* @brief Whether we have initialised all the necessary elements for the scanner to function */
	public: bool scanner_initialised{false};

	/* @brief The entity of this scanner. Set during Configure */
	public: gz::sim::Entity scanner_entity;
		
	/* @brief Model wrapper for scanner model */
	public: gz::sim::Model scanner_model;

	/* @brief Complete a round of RFID scanning and populate the RFIDScanResponse message */
	public: bool DoScan(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm, gz::custom_msgs::RFIDScanResponse& _reply);

	/* @brief Callback function for a request to do a scan */
	public: bool OnScanRequest(gz::custom_msgs::RFIDScanResponse& _reply);

	public:
		std::random_device rd;
		std::mt19937 gen;
		std::uniform_real_distribution<> distribution;

	/* @brief Queue of scan requests to be processed during PreUpdate */
	public: std::deque<PendingScan> pendingScans;

	/* @brief Mutex to control access to pending scan queue */
	public: std::mutex pendingMutex;

	// The following parameters are set via the SDF. Defaults are below.

	/* @brief Antenna transmission gain (in dB) */
	public: double antenna_power{30};

	/* @brief The path loss factor when we have line of sight to the tag */
	public: double path_loss_los_exponent{2.2};

	/* @brief Loss (in dB) with distance at 0m for a UHF reader (at ~900Mhz) */
	public: double path_loss_base_loss{31};

	/* @brief Minimum distance to use for path loss (avoids log singularities) */
	public: double path_loss_min_distance{0.2};

	/* @brief Maximum loss from polarization angle (in dB) */
	public: double polarization_max_loss{25};

	/* @brief Antenna (directional) gain peak (in dBi) */
	public: double antenna_gain_peak{6};

	/* @brief Antenna (directional) gain maximum loss (in dBi) */
	public: double antenna_gain_max_loss{25};

	/* @brief Antenna (directional) gain half beamwidth angle (in radians) */
	public: double antenna_gain_half_beamwidth{0.7};

	/* @brief Tag (directional) gain (in dBi) */
	public: double tag_directional_gain{0};

	/* @brief Constnat loss from tag reradiation (in dB) */
	public: double backscatter_loss{20};

	/* @brief Power threshold at which transmitted and received signals each have 50% read probability (for sigmoid) */
	public: double tx_threshold_power{-15};
	public: double rx_threshold_power{-70};

	/* @brief Scaling parameter for rx and tx read probabilities */
	public: double tx_read_scaling{2};
	public: double rx_read_scaling{2};

	/* @brief Whether we return all tags, regardless of RSSI. Used to delegate choice of 'successful read' to caller, or during tests. */
	public: bool return_all{false};

};

RFIDScannerPrivate::RFIDScannerPrivate() :
	gen(rd()),
	distribution(0.0,1.0)
{
}

RFIDScanner::RFIDScanner(void)
	: dataPtr(std::make_unique<RFIDScannerPrivate>())
{
}


bool RFIDScannerPrivate::OnScanRequest(gz::custom_msgs::RFIDScanResponse& _reply)
{
	if (!scanner_initialised) {
		gzwarn << "Scanner has not been initialised. Scan will not be completed\n";
		return false;
	}

	// Trigger a scan to run during PreUpdate by adding to queue and blocking until complete

	PendingScan scan;

	auto future = scan.promise.get_future();

	scan._reply = &_reply;

	{
		std::lock_guard<std::mutex> lock(pendingMutex);
		pendingScans.push_back(std::move(scan));
		gzdbg << "Added scan request to pendingScans queue\n";
	}

	// Wait until scan completes in PreUpdate, with 5 sec timeout
	if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
		return future.get();
	}

	return false;
}

bool RFIDScannerPrivate::DoScan(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm, gz::custom_msgs::RFIDScanResponse& _reply)
{
	// Fail if we have not yet initialised the scanner
	if (!scanner_initialised) {
		gzwarn << "Scan requested but scanner not yet initialised\n";
		return false;

	}

	// Add scan timestamp
	auto scan_time = _reply.mutable_time();

	scan_time->set_sec(
		(int32_t)(std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime).count())
	);
	
	scan_time->set_nsec(
		std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count()
	);

	// TODO We are unable to check whether this was successful. An error message is logged but that is it.
	// Retrieve current pose of the scanner
	gz::math::Pose3d sp = worldPose(scanner_entity, _ecm);

	// Retrieve scanner orientation
	gz::math::Quaterniond& scanner_orientation = sp.Rot();

	// Here we iterate over all tags found in the simulation and determine if they will be read during our scan
	_ecm.Each<gz::sim::components::RFIDTag>(
			[&](const gz::sim::Entity &_entity, const gz::sim::components::RFIDTag *_tag) -> bool {

				const auto &tag = _tag->Data();
				
				/* This section assumes our tag objects have the following structure:
				 *
				 * <model>
				 * 	<include>
				 * 		<uri>...
				 * 		<name>...
				 * 	</include>
				 * 	<plugin filename="librfidPlugin" name="RFIDTagPlugin">
				 * 		<uid>...</uid>
				 * 		<data>...</data>
				 * 	</plugin>
				 * </model>
				 *
				 */

				// TODO This does not check if we actually have a world pose
				// Retrieve pose of tag
				gz::math::Pose3d wp = gz::sim::worldPose(_entity, _ecm);

				// To estimate rssi (and calculate read probability), we need both the scanner pose and the tag pose
				//
				// From these, we calculate:
				// 1. Polarization:    difference in angle between tag orientation and scanner orientation
				// 2. Linear distance: difference in position between scanner and tag
				// 3. Antenna gain:    difference in angle between antenna boresight vector and scanner to tag vector

				// Vector from scanner to tag in world frame
				gz::math::Vector3d scanner_pose_vector = wp.CoordPositionSub(sp);

				double tag_scanner_linear_distance = scanner_pose_vector.Length();

				// This vector represents that orientation of the tag polarization vector in the tag frame
				gz::math::Vector3d tag_local_ref(1,0,0);

				// Scanner boresight points along +ve x-axis
				gz::math::Vector3d antenna_local_ref(1,0,0);

				gz::math::Quaterniond& tag_orientation = wp.Rot();

				double cos_theta = (tag_orientation.RotateVector(tag_local_ref)).Dot(scanner_orientation.RotateVector(antenna_local_ref));

				// Calculate dot product between scanner to tag vector and scanner orientation
				gz::math::Vector3d scanner_direction = scanner_orientation.RotateVector(antenna_local_ref);

				// Relative angle between antenna boresight and direction to tag
				double antenna_gain_angle = std::acos(scanner_direction.Dot(scanner_pose_vector.Normalized()));

				// Calculate tranmission power
				double antenna_gain = antenna_gain_peak - std::min(antenna_gain_max_loss, 12*std::pow(antenna_gain_angle / antenna_gain_half_beamwidth,2));

				// Calculate polarization loss
				//
				// In default configuration, equation is: polarization_loss = min(25, log10(cos(\theta)))
				double loss_polarization = std::min(polarization_max_loss, -20*std::log10(std::abs(cos_theta)));

				// TODO Add LOS / NLOS factor between reader and tag. We assume no line-of-sight always here, hence the multiplication by path_loss_los_exponent
				
				// Calculate path loss
				//
				// In default configuration, equation is: path_loss = 31 + 10*2.2*los10(max(0.2, distance))
				double loss_path = path_loss_base_loss + 10*path_loss_los_exponent*std::log10(std::max(path_loss_min_distance, tag_scanner_linear_distance));

				// Calculate tranmission and received power
				double transmission_power = antenna_power + antenna_gain + 2*tag_directional_gain - loss_polarization - loss_path;
				double received_power = antenna_power + 2*antenna_gain + 2*tag_directional_gain - 2*(loss_polarization + loss_path) - backscatter_loss;

				// Calculate read probabilities
				double p_read_tx = sigmoid((transmission_power - tx_threshold_power) / tx_read_scaling);
				double p_read_rx = sigmoid((received_power - rx_threshold_power) / rx_read_scaling);
				double p_read = p_read_tx*p_read_rx;

				// DEBUG
				gzdbg << scanner_model.Name(_ecm) << "\n";
				gzdbg << "    Scanner Pose3d     (m,rad): " << sp << "\n";
				gzdbg << "    Tag Pose3d         (m,rad): " << wp << "\n";
				gzdbg << "    Scanner-Tag Pose3d (m,rad): " << scanner_pose_vector << "\n";
				gzdbg << "    Scanner Quaternion        : " << sp.Rot() << "\n";
				gzdbg << "    Scanner Eul               : " << sp.Rot().Euler() << "\n";
				gzdbg << "    Tag Quaternion            : " << wp.Rot() << "\n";
				gzdbg << "    Tag Eul                   : " << wp.Rot().Euler() << "\n";
				gzdbg << "    Scanner-Tag Vec        (m): " << scanner_pose_vector << "\n";
				gzdbg << "    Polarization angle   (rad): " << std::acos(cos_theta) << "\n";
				gzdbg << "    Scanner direction vec  (m): " << scanner_direction << "\n";
				gzdbg << "    Antenna gain angle (cos(rad)): " << antenna_gain_angle << "\n";
				gzdbg << "    Linear distance   : " << tag_scanner_linear_distance << "\n";
				gzdbg << " Transmission Stats\n";
				gzdbg << "	   antenna gain: " << antenna_gain << "\n";
				gzdbg << "	   loss polarization: " << loss_polarization << "\n";
				gzdbg << "	   loss path: " << loss_path << "\n";
				gzdbg << "	   transmission power: " << transmission_power << "\n";
				gzdbg << "	   received power: " << received_power << "\n";
				gzdbg << " Read Stats\n";
				gzdbg << "    tag read prob: " << p_read_tx << "\n";
				gzdbg << "    antenna read prob: " << p_read_rx << "\n";
				gzdbg << "    total read prob: " << p_read << "\n";

				// Sample from distribution to determine if read was successful (or if we have configured to return all tags)
				if ( return_all || distribution(gen) < p_read ) {

					// Successful read - add tag to list of found tags
					auto* scanmessage = _reply.add_scan();

					scanmessage->set_uid(tag.uid);
					scanmessage->set_data(tag.data);
					scanmessage->set_rssi(received_power);
				}

				return true;
			});
	
	return true;
}


void RFIDScanner::Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr)
{
	// Get reference to parent model of RFIDScanner
	this->dataPtr->scanner_model = gz::sim::Model(_entity);

	// Error if parent is not <model>
	if (!this->dataPtr->scanner_model.Valid(_ecm)) {
		gzwarn << "Parent of RFID scanner is not a <model> element. Failed to configure\n";
		return;
	}
	
	// Store the entity
	this->dataPtr->scanner_entity = _entity;

	// Expose the scan service, prefixed by the scanner model name
	if (!this->dataPtr->node.Advertise<class RFIDScannerPrivate,
			gz::custom_msgs::RFIDScanResponse>(
				this->dataPtr->scanner_model.Name(_ecm) + "/scan_request",
				&RFIDScannerPrivate::OnScanRequest,
				this->dataPtr.get())
			) {
		gzwarn << "Failed to create RFID scan service\n";
		return;
	} else {
		gzdbg << "Created RFID scan service\n";
	}

	// Retrieve configuration from the sdf
	if (_sdf->HasElement("antenna_power")) this->dataPtr->antenna_power = _sdf->Get<double>("antenna_power");

	if (_sdf->HasElement("path_loss_los_exponent")) this->dataPtr->path_loss_los_exponent = _sdf->Get<double>("path_loss_los_exponent");

	if (_sdf->HasElement("path_loss_base_loss")) this->dataPtr->path_loss_base_loss = _sdf->Get<double>("path_loss_base_loss");

	if (_sdf->HasElement("path_loss_min_distance")) {
		double min_distance = _sdf->Get<double>("path_loss_min_distance");

		if (min_distance >= 0.0) {
			this->dataPtr->path_loss_min_distance = min_distance;
		} else {
			gzwarn << "path_loss_min_distance must be greater than 0. Using default.\n";
		}
	}

	if (_sdf->HasElement("polarization_max_loss")) this->dataPtr->polarization_max_loss = _sdf->Get<double>("polarization_max_loss");

	if (_sdf->HasElement("antenna_gain_peak")) this->dataPtr->antenna_gain_peak = _sdf->Get<double>("antenna_gain_peak");

	if (_sdf->HasElement("antenna_gain_max_loss")) this->dataPtr->antenna_gain_max_loss = _sdf->Get<double>("antenna_gain_max_loss");

	if (_sdf->HasElement("antenna_gain_half_beamwidth")) {
		double h_beamwidth = _sdf->Get<double>("antenna_gain_half_beamwidth");
		if (h_beamwidth >= 0.0) {
			this->dataPtr->antenna_gain_half_beamwidth = h_beamwidth;
		} else {
			gzwarn << "Half beamwidth angle must be greater than 0. Using default.\n";
		}
	}

	if (_sdf->HasElement("tag_directional_gain")) this->dataPtr->tag_directional_gain = _sdf->Get<double>("tag_directional_gain");

	if (_sdf->HasElement("backscatter_loss")) this->dataPtr->backscatter_loss = _sdf->Get<double>("backscatter_loss");

	if (_sdf->HasElement("tx_threshold_power")) this->dataPtr->tx_threshold_power = _sdf->Get<double>("tx_threshold_power");

	if (_sdf->HasElement("rx_threshold_power")) this->dataPtr->rx_threshold_power = _sdf->Get<double>("rx_threshold_power");

	if (_sdf->HasElement("tx_read_scaling")) {
		double tx_read_s = _sdf->Get<double>("tx_read_scaling");
		if (tx_read_s > 0.0) {
			this->dataPtr->tx_read_scaling = tx_read_s;
		} else {
			gzwarn << "tx_read_scaling must be greater than 0. Using default.\n";
		}
	}

	if (_sdf->HasElement("rx_read_scaling")) {
		double rx_read_s = _sdf->Get<double>("rx_read_scaling");
		if (rx_read_s > 0.0) {
			this->dataPtr->rx_read_scaling = rx_read_s;
		} else {
			gzwarn << "rx_read_scaling must be greater than 0. Using default.\n";
		}
	}

	if (_sdf->HasElement("return_all")) this->dataPtr->return_all = _sdf->Get<bool>("return_all");

	this->dataPtr->scanner_initialised = true;

	return;
}

void RFIDScanner::PreUpdate(const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm)
{
	// Return if we have not yet initialised
	if (!this->dataPtr->scanner_initialised) {
		return;
	}

	// Retrieve next remaining scan to be processed. Process at most 1 per PreUpdate.
	PendingScan scan;

	{
		std::lock_guard<std::mutex> lock(this->dataPtr->pendingMutex);

		if (this->dataPtr->pendingScans.empty()) return;

		scan = std::move(this->dataPtr->pendingScans.front());
		this->dataPtr->pendingScans.pop_front();
	}

	// Execute scan
	bool success = this->dataPtr->DoScan(_info, _ecm, *(scan._reply));
	scan.promise.set_value(success);

	return;
}

GZ_ADD_PLUGIN(RFIDScanner,
		gz::sim::System,
		RFIDScanner::ISystemConfigure,
		RFIDScanner::ISystemPreUpdate
)
