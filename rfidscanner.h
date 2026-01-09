#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/transport/Node.hh>

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

		/* @brief Number of iterations of PreUpdate since last scan */
		uint16_t scan_index{0};

	private:
		/* @brief Flag for whether we need to a do a scan during next PreUpdate */
		bool do_scan_flag{false};

		/* @brief Callback function for a request to do a scan */
		bool scanRequestCallback(gz::custom_msgs::RFIDScanResponse& _reply);

	private:
		// Configuration parameters (set via SDF)
		
		/* @brief Frequency that we update our scanned tag list */
		uint16_t scan_period{10};


};
