set(TEST_TYPE "INTEGRATION")

gz_build_tests(TYPE INTEGRATION
	SOURCES gazebo_rfid_plugin.cc
	LIB_DEPS rfidScanner rfidTag gz-sim8 gz-common5 ${PROJECT_NAME}-msgs
)
		
target_compile_definitions(INTEGRATION_gazebo_rfid_plugin PRIVATE
	PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

