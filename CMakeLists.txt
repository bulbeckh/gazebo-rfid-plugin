cmake_minimum_required(VERSION 3.10.2)

project(Gazebo-RFID-Plugin)


find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

## Add custom RFID messages
set(MSGS_PROTOS
	${CMAKE_SOURCE_DIR}/proto/gz/custom_msgs/rfid_create_request.proto
	${CMAKE_SOURCE_DIR}/proto/gz/custom_msgs/rfid_create_response.proto
	${CMAKE_SOURCE_DIR}/proto/gz/custom_msgs/rfid_scan_response.proto
)

gz_msgs_generate_messages(
	TARGET msgs
	PROTO_PACKAGE "gz.custom_msgs"
	MSGS_PATH ${CMAKE_SOURCE_DIR}/proto
	MSGS_PROTOS ${MSGS_PROTOS}
	DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)


add_library(rfidPlugin
	SHARED
	rfidmanager.cpp
	rfidscanner.cpp
)

target_link_libraries(rfidPlugin PRIVATE
	gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
	${PROJECT_NAME}-msgs
)



