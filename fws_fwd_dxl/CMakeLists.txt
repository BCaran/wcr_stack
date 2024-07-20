cmake_minimum_required(VERSION 3.5)
project(fws_fwd_dxl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)

add_executable(fwsfwd_controller src/fwsfwd_controller.cpp)
ament_target_dependencies(fwsfwd_controller rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros std_srvs dynamixel_workbench_toolbox)

install(TARGETS
  fwsfwd_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
