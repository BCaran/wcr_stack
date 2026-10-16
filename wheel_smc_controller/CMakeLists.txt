cmake_minimum_required(VERSION 3.16)
project(wheel_smc_controller LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
set(THIS_PACKAGE_DEPENDENCIES
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  nav_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep ${THIS_PACKAGE_DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)

# ----------------------------------------------------------------------------
# Library
# ----------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/wheel_smc_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_DEPENDENCIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE
  "WHEEL_SMC_CONTROLLER_BUILDING_DLL"
)

# Export plugin to pluginlib
pluginlib_export_plugin_description_file(controller_interface wheel_smc_controller.xml)

# ----------------------------------------------------------------------------
# Install
# ----------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Tests
# ----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_load_wheel_smc_controller
    test/test_load_wheel_smc_controller.cpp
  )
  target_link_libraries(test_load_wheel_smc_controller ${PROJECT_NAME})
  ament_target_dependencies(test_load_wheel_smc_controller
    controller_interface
    pluginlib
    rclcpp
  )
endif()

# ----------------------------------------------------------------------------
# Export
# ----------------------------------------------------------------------------
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES} Eigen3)
ament_package()