/* Test Suite for RFID Plugin
 *
 * 1. Expected RSSI Default- with scanner at zero-pose, do we get the expected read strength from each tag in a set of tags about origin
 *
 * 2. Expected RSSI (different distance scaling)
 * 3. Expected RSSI (different polarization scaling)
 * 4. Expected RSSI (different antenna gain)
 *
 * REMOVED Service call test
 * REMOVED Maximum distance test (no read of tag beyond set maximum distance)
 * REMOVED Tag ID and data correctness
 *
 * 6. Duplicate scanner names produces error (would produce same service name)
 *
 *
 * Timing tests or benchmarks?? i.e. time taken to process 500 tags, 5000 tags, etc.
 *
 *
 * Add tests for moving tags - does read rssi change after tag move
 *
 * Add tests for improper sdf configuration (like scanner not under a model)
 *
 *
 */

#include <gtest/gtest.h>

#include <gz/sim/Server.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/transport/Node.hh>

#include <gz/common/Util.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <gz/custom_msgs/rfid_scan_response.pb.h>
#include <gz/msgs/empty.pb.h>

#define TEST_TOL 1e-4

using namespace gz;
using namespace sim;

class RFIDPluginTestFixture : public testing::Test
{

};

TEST_F(RFIDPluginTestFixture, ExpectedRSSI)
{
	std::map<std::string, double> rssi_expected{
		{"tag0", -95.19690470741703},
		{"tag1", -107.11932750844016},
		{"tag2", -112.91063907348095},
		{"tag3", -117.61599511227273},
		{"tag4", -121.49000281149772},
		{"tag5", -107.11932750844016},
		{"tag6", -110.99333520766515},
		{"tag7", -115.21481678492124},
		{"tag8", -119.08882448414623},
		{"tag9", -122.49666760383258},
		{"tag10", -112.91063907348095},
		{"tag11", -115.21481678492124},
		{"tag12", -118.13257922096224},
		{"tag13", -121.129413655358},
		{"tag14", -123.96949697570606},
		{"tag15", -117.61599511227273},
		{"tag16", -119.08882448414623},
		{"tag17", -121.129413655358},
		{"tag18", -123.40730667731353},
		{"tag19", -125.7114843887538},
		{"tag20", -121.49000281149772},
		{"tag21", -122.49666760383258},
		{"tag22", -123.96949697570606},
		{"tag23", -125.7114843887538},
		{"tag24", -127.5665438741546},
		{"tag25", -75.67354490199044},
		{"tag26", -95.6413274661585},
		{"tag27", -114.86809077051639},
		{"tag28", -119.08882448414623},
		{"tag29", -122.49666760383258},
		{"tag30", -95.6413274661585},
		{"tag31", -108.4422245166322},
		{"tag32", -117.06987627032203},
		{"tag33", -120.36464731765533},
		{"tag34", -123.40730667731353},
		{"tag35", -114.86809077051639},
		{"tag36", -117.06987627032203},
		{"tag37", -119.53329897808854},
		{"tag38", -122.17275595377703},
		{"tag39", -124.75523912556983},
		{"tag40", -119.08882448414623},
		{"tag41", -120.36464731765533},
		{"tag42", -122.17275595377703},
		{"tag43", -124.23865501688033},
		{"tag44", -126.37067530305757},
		{"tag45", -122.49666760383258},
		{"tag46", -123.40730667731353},
		{"tag47", -124.75523912556983},
		{"tag48", -126.37067530305757},
		{"tag49", -128.11266271610532},
		{"tag50", -72.41511144348124},
		{"tag51", -85.30961677056362},
		{"tag52", -100.40519750067455},
		{"tag53", -114.5719821275864},
		{"tag54", -123.96949697570606},
		{"tag55", -85.30961677056362},
		{"tag56", -95.06121093954673},
		{"tag57", -107.18909462037296},
		{"tag58", -119.20541959819357},
		{"tag59", -124.75523912556983},
		{"tag60", -100.40519750067455},
		{"tag61", -107.18909462037296},
		{"tag62", -116.19023991508219},
		{"tag63", -123.69253617492961},
		{"tag64", -125.9363060227509},
		{"tag65", -114.5719821275864},
		{"tag66", -119.20541959819357},
		{"tag67", -123.69253617492961},
		{"tag68", -125.48124484783418},
		{"tag69", -127.37734009539241},
		{"tag70", -123.96949697570606},
		{"tag71", -124.75523912556983},
		{"tag72", -125.9363060227509},
		{"tag73", -127.37734009539241},
		{"tag74", -128.95874425612718},
		{"tag75", -73.27260531769292},
		{"tag76", -81.81541420701551},
		{"tag77", -93.04841443879246},
		{"tag78", -104.79751902561128},
		{"tag79", -115.88022817251414},
		{"tag80", -81.81541420701551},
		{"tag81", -88.9188647112056},
		{"tag82", -98.53070983427678},
		{"tag83", -108.88664727537368},
		{"tag84", -118.9092416227337},
		{"tag85", -93.04841443879246},
		{"tag86", -98.53070983427678},
		{"tag87", -106.21218859824332},
		{"tag88", -114.80770618102434},
		{"tag89", -123.41778968945941},
		{"tag90", -104.79751902561128},
		{"tag91", -108.88664727537368},
		{"tag92", -114.80770618102434},
		{"tag93", -121.68754432584738},
		{"tag94", -128.62924682479482},
		{"tag95", -115.88022817251414},
		{"tag96", -118.9092416227337},
		{"tag97", -123.41778968945941},
		{"tag98", -128.62924682479482},
		{"tag99", -130.02996658192112},
		{"tag100", -75.21142090585421},
		{"tag101", -81.15866566845591},
		{"tag102", -89.54654037545151},
		{"tag103", -99.00192105567331},
		{"tag104", -108.53865923202886},
		{"tag105", -81.15866566845591},
		{"tag106", -86.38813140025985},
		{"tag107", -93.87282589472356},
		{"tag108", -102.4524280669352},
		{"tag109", -111.24517749786123},
		{"tag110", -89.54654037545151},
		{"tag111", -93.87282589472356},
		{"tag112", -100.18648269833756},
		{"tag113", -107.59041479260475},
		{"tag114", -115.350058130071},
		{"tag115", -99.00192105567331},
		{"tag116", -102.4524280669352},
		{"tag117", -107.59041479260475},
		{"tag118", -113.7640851707153},
		{"tag119", -120.39676527013542},
		{"tag120", -108.53865923202886},
		{"tag121", -111.24517749786123},
		{"tag122", -115.350058130071},
		{"tag123", -120.39676527013542},
		{"tag124", -125.95158489820187},
	};

	ServerConfig serverConfig;
	serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_DIR) + "/test/integration/worlds/rssi_test_default.sdf");

	Server server(serverConfig);

	transport::Node node;

	//size_t iters100 = 100u;
	//server.Run(true, iters100);
	
	server.Run(false, 0, true);

	gz::custom_msgs::RFIDScanResponse rep;
	gz::msgs::Empty req;
	bool result = false;

	node.Request("/scanner/scan_request",
			req,
			1000,
			rep,
			result);

	EXPECT_TRUE(result);

	// Check that our RSSI from each tag is what is expected from a tag at that pose relative to scanner
	for(auto g : rep.scan()) {
		if (rssi_expected.find(g.uid()) != rssi_expected.end() ) {
			EXPECT_NEAR(g.rssi(), rssi_expected[g.uid()], TEST_TOL);

			// Assert that data is correct here
		} else {
			EXPECT_FALSE(true);
		}
	}

}

