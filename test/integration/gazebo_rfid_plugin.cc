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
		{"tag0", -41.97244664902168},
		{"tag1", -51.05794340312833},
		{"tag2", -58.5046882386425},
		{"tag3", -64.27501304869871},
		{"tag4", -68.86905767322217},
		{"tag5", -51.05794340312833},
		{"tag6", -56.0849017305048},
		{"tag7", -61.35866354521818},
		{"tag8", -66.03692366498962},
		{"tag9", -70.04248274183837},
		{"tag10", -58.5046882386425},
		{"tag11", -61.35866354521818},
		{"tag12", -64.89523092605351},
		{"tag13", -68.44679165522894},
		{"tag14", -71.74546304149592},
		{"tag15", -64.27501304869871},
		{"tag16", -66.03692366498962},
		{"tag17", -68.44679165522894},
		{"tag18", -71.09730946978087},
		{"tag19", -73.73967960965335},
		{"tag20", -68.86905767322217},
		{"tag21", -70.04248274183837},
		{"tag22", -71.74546304149592},
		{"tag23", -73.73967960965335},
		{"tag24", -75.84120612397811},
		{"tag25", -45.39221913915007},
		{"tag26", -51.23771425933059},
		{"tag27", -57.297342848156646},
		{"tag28", -62.60397787272269},
		{"tag29", -67.093581242695},
		{"tag30", -51.23771425933059},
		{"tag31", -55.21776645823684},
		{"tag32", -59.8669433058229},
		{"tag33", -64.3032632123435},
		{"tag34", -68.26748168734694},
		{"tag35", -57.297342848156646},
		{"tag36", -59.8669433058229},
		{"tag37", -63.19857306870643},
		{"tag38", -66.6734969396685},
		{"tag39", -69.98640030517046},
		{"tag40", -62.60397787272269},
		{"tag41", -64.3032632123435},
		{"tag42", -66.6734969396685},
		{"tag43", -69.33022153971997},
		{"tag44", -72.01838582689902},
		{"tag45", -67.093581242695},
		{"tag46", -68.26748168734694},
		{"tag47", -69.98640030517046},
		{"tag48", -72.01838582689902},
		{"tag49", -74.17719867610155},
		{"tag50", -50.07493693880598},
		{"tag51", -53.67642978316243},
		{"tag52", -58.085974960227},
		{"tag53", -62.45112829320597},
		{"tag54", -66.44647622627377},
		{"tag55", -53.67642978316243},
		{"tag56", -56.49881476730205},
		{"tag57", -60.14613394426838},
		{"tag58", -63.93425725021805},
		{"tag59", -67.52904740859498},
		{"tag60", -58.085974960227},
		{"tag61", -60.14613394426838},
		{"tag62", -62.96578185668682},
		{"tag63", -66.06259460275949},
		{"tag64", -69.1385982569436},
		{"tag65", -62.45112829320597},
		{"tag66", -63.93425725021805},
		{"tag67", -66.06259460275949},
		{"tag68", -68.5209194151168},
		{"tag69", -71.07467948107322},
		{"tag70", -66.44647622627377},
		{"tag71", -67.52904740859498},
		{"tag72", -69.1385982569436},
		{"tag73", -71.07467948107322},
		{"tag74", -73.1658160311994},
		{"tag75", -54.3089298624367},
		{"tag76", -56.64783172519772},
		{"tag77", -59.81449125132872},
		{"tag78", -63.25260768998},
		{"tag79", -66.63215550226444},
		{"tag80", -56.64783172519772},
		{"tag81", -58.63753894836523},
		{"tag82", -61.40160530413824},
		{"tag83", -64.48303406854576},
		{"tag84", -67.5816496772179},
		{"tag85", -59.81449125132872},
		{"tag86", -61.40160530413824},
		{"tag87", -63.676193596785545},
		{"tag88", -66.29873636114996},
		{"tag89", -69.01729517066562},
		{"tag90", -63.25260768998},
		{"tag91", -64.48303406854576},
		{"tag92", -66.29873636114996},
		{"tag93", -68.46308626745203},
		{"tag94", -70.77916781461325},
		{"tag95", -66.63215550226444},
		{"tag96", -67.5816496772179},
		{"tag97", -69.01729517066562},
		{"tag98", -70.77916781461325},
		{"tag99", -72.72066057234403},
		{"tag100", -57.94587652805639},
		{"tag101", -59.55776236674893},
		{"tag102", -61.87768479217488},
		{"tag103", -64.56456288045143},
		{"tag104", -67.36062800549419},
		{"tag105", -59.55776236674893},
		{"tag106", -60.99745770587445},
		{"tag107", -63.09714350479116},
		{"tag108", -65.56569001663257},
		{"tag109", -68.17139577687041},
		{"tag110", -61.87768479217488},
		{"tag111", -63.09714350479116},
		{"tag112", -64.90695266551026},
		{"tag113", -67.07844174580092},
		{"tag114", -69.4166802056853},
		{"tag115", -64.56456288045143},
		{"tag116", -65.56569001663257},
		{"tag117", -67.07844174580092},
		{"tag118", -68.93326452217008},
		{"tag119", -70.97489942122736},
		{"tag120", -67.36062800549419},
		{"tag121", -68.17139577687041},
		{"tag122", -69.4166802056853},
		{"tag123", -70.97489942122736},
		{"tag124", -72.7271268398065}
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

