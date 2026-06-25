cmake_minimum_required(VERSION 3.10.2)

project(gazebo_rfid_plugin)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

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
	${CMAKE_CURRENT_SOURCE_DIR}/proto/gz/custom_msgs/rfid_scan_response.proto
)

gz_msgs_generate_messages(
	TARGET msgs
	PROTO_PACKAGE "gz.custom_msgs"
	MSGS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/proto
	MSGS_PROTOS ${MSGS_PROTOS}
	DEPENDENCIES gz-msgs${GZ_MSGS_VER}::gz-msgs${GZ_MSGS_VER}
)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/ScanResponse.msg"
	"msg/RFIDScanResponse.msg"
	"srv/RFIDScan.srv"
	DEPENDENCIES
	std_msgs
	builtin_interfaces
)

add_library(rfidScanner SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/src/rfidscanner.cpp
)

add_library(rfidTag SHARED
	${CMAKE_CURRENT_SOURCE_DIR}/src/rfidtag.cpp
)

target_link_libraries(rfidScanner PRIVATE
	gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
	${PROJECT_NAME}-msgs
)

target_link_libraries(rfidTag PRIVATE
	gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
	${PROJECT_NAME}-msgs
)

## Testing
add_subdirectory(test/integration)

install(DIRECTORY rfid-scanner
	DESTINATION share/${PROJECT_NAME}
)

install(TARGETS rfidScanner rfidTag
	LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}-msgs
	EXPORT export_${PROJECT_NAME}-msgs
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-msgs.gz_desc
	DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
	"${CMAKE_CURRENT_SOURCE_DIR}/hooks/gazebo_rfid_plugin_pkg.dsv.in"
	"${CMAKE_CURRENT_SOURCE_DIR}/hooks/gazebo_rfid_plugin_pkg.sh.in"
)

ament_export_targets(export_${PROJECT_NAME}-msgs HAS_LIBRARY_TARGET)
ament_export_dependencies(gz-msgs10)

ament_export_dependencies(rosidl_default_runtime std_msgs builtin_interfaces)

ament_package()

