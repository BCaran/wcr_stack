cmake_minimum_required(VERSION 3.8)
project(wcr_teensy_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wcr_interfaces REQUIRED)

add_executable(teensy_bridge_node src/teensy_bridge_node.cpp)
ament_target_dependencies(teensy_bridge_node
  rclcpp
  rcl_interfaces
  std_msgs
  std_srvs
  wcr_interfaces
)

target_link_libraries(teensy_bridge_node pthread)

install(TARGETS teensy_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
