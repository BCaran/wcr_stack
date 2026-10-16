cmake_minimum_required(VERSION 3.16)
project(swerve_drive_controller LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)

generate_parameter_library(swerve_drive_controller_parameters
  src/swerve_drive_controller_parameters.yaml
)

# ----------------------------------------------------------------------------
# Library
# ----------------------------------------------------------------------------
add_library(swerve_drive_controller SHARED
  src/swerve_drive_controller.cpp
  src/swerve_kinematics.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(swerve_drive_controller PUBLIC cxx_std_17)
target_include_directories(swerve_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/swerve_drive_controller>
)
target_link_libraries(swerve_drive_controller PUBLIC
  swerve_drive_controller_parameters
  Eigen3::Eigen
)
ament_target_dependencies(swerve_drive_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(swerve_drive_controller PRIVATE "SWERVE_DRIVE_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface swerve_drive_controller.xml)

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)

  ament_add_gmock(test_swerve_kinematics
    test/test_swerve_kinematics.cpp)
  target_link_libraries(test_swerve_kinematics swerve_drive_controller)

  ament_add_gmock(test_swerve_drive_controller
    test/test_swerve_drive_controller.cpp)
  target_link_libraries(test_swerve_drive_controller swerve_drive_controller)
  ament_target_dependencies(test_swerve_drive_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_srvs
    tf2
    tf2_msgs
  )

  add_rostest_with_parameters_gmock(test_load_swerve_drive_controller
    test/test_load_swerve_drive_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_swerve_drive_controller.yaml
  )
  ament_target_dependencies(test_load_swerve_drive_controller
    controller_manager
  )
endif()

# ----------------------------------------------------------------------------
# Install
# ----------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include/swerve_drive_controller
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS swerve_drive_controller swerve_drive_controller_parameters
  EXPORT export_swerve_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_swerve_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3)
ament_package()
