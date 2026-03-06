cmake_minimum_required(VERSION 3.10.2)

project(Gazebo-RFID-Plugin)

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})

gz_find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

## Add custom RFID messages
set(MSGS_PROTOS
	${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/custom_msgs/rfid_tag_list.proto
	${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/custom_msgs/rfid_scan_response.proto
)

gz_msgs_generate_messages(
	TARGET msgs
	PROTO_PACKAGE "gz.custom_msgs"
	MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
	MSGS_PROTOS ${MSGS_PROTOS}
	DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

add_library(rfidPlugin SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/rfidmanager.cpp
)

add_library(rfidScanner SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/rfidscanner.cpp
)

target_link_libraries(rfidPlugin PRIVATE
	gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
	${PROJECT_NAME}-msgs
)

target_link_libraries(rfidScanner PRIVATE
	gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
	${PROJECT_NAME}-msgs
)

