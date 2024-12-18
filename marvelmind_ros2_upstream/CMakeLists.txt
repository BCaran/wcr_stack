cmake_minimum_required(VERSION 3.8)
project(marvelmind_ros2)

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

# Find needed packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marvelmind_ros2_msgs REQUIRED)
find_package(rosidl_default_generators 	REQUIRED)

include_directories(include)

add_library(marvelmind_hedge_lib
  src/marvelmind_hedge.c
)

add_library(marvelmind_api_lib
  src/marvelmind_api.c
)


# Find src files to compile
add_executable(marvelmind_ros2 src/marvelmind_ros2.cpp)
ament_target_dependencies(marvelmind_ros2 rclcpp std_msgs geometry_msgs marvelmind_ros2_msgs)

add_executable(marvelmind_api_ros2 src/marvelmind_api_ros2.cpp)
ament_target_dependencies(marvelmind_api_ros2 rclcpp std_msgs geometry_msgs marvelmind_ros2_msgs)

target_link_libraries(marvelmind_ros2
  marvelmind_hedge_lib
)

target_link_libraries(marvelmind_api_ros2
  marvelmind_api_lib
)

install( 
  TARGETS marvelmind_ros2
  DESTINATION lib/${PROJECT_NAME}
)

install( 
  TARGETS marvelmind_api_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# make sure we grab config folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
