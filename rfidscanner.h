#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/transport/Node.hh>

#include <chrono>
#include <random>

#include <gz/custom_msgs/rfid_scan_response.pb.h>

#include <sdf/sdf.hh>

/* This RFID Scanner class has a number of configuration parameters that need
 * to be set via the SDF.
 *
 *
 * TODO Add SDF example here
 *
 *
 * Something that we need to deal with is that the scan requests are triggered
 * via a service callback, which is independent of the PreUpdate function. Additionally,
 * we do not have access to the _ecm from the callback and so we cannot access the state
 * of the world during the callback. NOTE I don't believe it is safe or correct to store
 * the _ecm reference for use outside of the PreUpdate but I may be wrong and if so it would
 * simplify our approach.
 *
 * One workaround is to do a scan every PreUpdate (or perhaps every nth iteration) and
 * store the result. When the callback is triggered, we simply retrieve the most recent
 * scan and then send as a ScanResponse message.
 *
 */
class RFIDScannerPlugin :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{

	public:
		RFIDScannerPlugin();

	public:
		void Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr) final;

		void PreUpdate(const gz::sim::UpdateInfo &_info,
					gz::sim::EntityComponentManager &_ecm) final;

	private:
		/* @brief Node for the scan service */
		gz::transport::Node node;

		// TODO There is a better way to identify models (entities) as tags than check the name. Perhaps use components
		/* @brief The prefix used to identify a model as a tag model */
		std::string tag_prefix{"rfid-tag-"};

		/* @brief Service name to use for scan request */
		std::string scan_service_name{"scan_request"};

		/* @brief Whether we have initialised all the necessary elements for the scanner to function */
		bool scanner_initialised{false};

		/* @brief The entity of this scanner. Set during Configure */
		gz::sim::Entity scanner_entity;

		/* @brief The canonical link of the scanner that is used to determine it's pose. Set during first PreUpdate */
		gz::sim::Link scanner_link;

		/* @brief A pointer to the ECM. NOTE We can do this as long as we don't use it to modify the ecm (that should be done
		 * during PreUpdate/PostUpdate) */
		gz::sim::EntityComponentManager* ecm_internal{nullptr};
		
		/* @brief We keep track of the current simulation time so that we can report it during a scan request */
		int64_t simulation_time_sec;
		int32_t simulation_time_nsec;

	private:
		/* @brief Flag for whether we need to a do a scan during next PreUpdate */
		bool do_scan_flag{false};

		/* @brief Callback function for a request to do a scan */
		bool scanRequestCallback(gz::custom_msgs::RFIDScanResponse& _reply);

	private:
		// Configuration parameters (set via SDF)

		/* @brief The path loss factor when we have line of sight to the tag */
		double path_loss_los_gain{2.2};

		/* @brief The path loss factor when we do not have lin of sight to the tag */
		double path_loss_nlos_gain{3};

		/* @brief Path loss scaling factor */
		double path_loss_scaling{5};

		/* @brief The minimum cos(\theta) value that we should have (avoids excessive loss from polarization) */
		double polarization_minimum_angle{0.05};

		/* @brief Maximum loss from polarization angle (in dB) */
		double polarization_max_loss{25};

		/* @brief Antenna transmission gain (in dB) */
		double antenna_power{30};

		// TODO This will become dependent on the direction of the antenna
		/* @brief Antenna (directional) gain */
		double antenna_directional_gain{6};

		/* @brief Tag (directional) gain */
		double tag_directional_gain{0};

		/* @brief Power threshold at which transmit and received waves each have 50% read probability (for sigmoid) */
		double tx_threshold_power{30};
		double rx_threshold_power{30};

		/* @brief Scaling parameter for rx and tx read probabilities */
		double tx_read_scaling{1};
		double rx_read_scaling{1};

		// TODO Add LOS static terms
	
	private:
		/* @brief Sigmoid function implementation */
		double sigmoid(double x);

		std::random_device rd;

		std::mt19937 gen;

		std::uniform_real_distribution<> distribution;

};
