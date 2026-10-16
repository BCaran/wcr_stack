// Copyright 2026 Branimir Caran
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"

namespace
{
// ros2_control_test_assets::diffbot_urdf references test hardware plugins
// whose names differ between ros2_control releases, so the load test uses
// mock_components with the WCR joints instead.
std::string joint(const std::string & name, const std::string & command)
{
  return "<joint name=\"" + name + "\">"
         "<command_interface name=\"" + command + "\"/>"
         "<state_interface name=\"position\"/>"
         "<state_interface name=\"velocity\"/>"
         "</joint>";
}

const std::string wcr_mock_urdf =
  std::string(R"(<?xml version="1.0"?><robot name="wcr"><link name="base_link"/>)") +
  R"(<ros2_control name="WCRMock" type="system"><hardware>)" +
  R"(<plugin>mock_components/GenericSystem</plugin></hardware>)" +
  joint("FL_wheel", "velocity") + joint("BL_wheel", "velocity") + joint("BR_wheel", "velocity") +
  joint("FR_wheel", "velocity") + joint("FL_steering", "position") +
  joint("BL_steering", "position") + joint("BR_steering", "position") +
  joint("FR_steering", "position") + "</ros2_control></robot>";
}  // namespace

TEST(TestLoadSwerveDriveController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(wcr_mock_urdf), executor,
    "test_controller_manager");

  ASSERT_NE(
    cm.load_controller(
      "test_swerve_drive_controller", "swerve_drive_controller/SwerveDriveController"),
    nullptr);

  ASSERT_EQ(
    cm.configure_controller("test_swerve_drive_controller"),
    controller_interface::return_type::OK);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
