cmake_minimum_required(VERSION 3.5)
project(marvelmind_ros2_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake 		REQUIRED)
find_package(rosidl_default_generators 	REQUIRED)
find_package(std_msgs 			REQUIRED)
find_package(geometry_msgs  REQUIRED)

set(MSG_DEPS std_msgs geometry_msgs)

set(MSG_FILES
  "msg/BeaconDistance.msg"
  "msg/BeaconPositionAddressed.msg"
  "msg/HedgeImuFusion.msg"
  "msg/HedgeImuRaw.msg"
  "msg/HedgePositionAddressed.msg"
  "msg/HedgePositionAngle.msg"
  "msg/HedgePosition.msg"
  "msg/HedgeQuality.msg"
  "msg/HedgeTelemetry.msg"
  "msg/MarvelmindWaypoint.msg"
  "msg/MarvelmindUserData.msg"
   "srv/MarvelmindAPI.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
