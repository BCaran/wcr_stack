// Verifies the controller is discoverable by pluginlib.
// We deliberately avoid spinning up a ResourceManager / minimal_robot_urdf
// because the test_assets URDF in some ros2_control releases references
// hardware plugin names that may not match what's installed locally
// (e.g. "test_actuator" vs "test_hardware_components/TestSingleJointActuator").
// The thing we actually want to verify here is that pluginlib can find and
// load our controller class — that's what catches XML/macro/install mistakes.

#include <gmock/gmock.h>

#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

TEST(TestLoadWheelSmcController, plugin_is_discoverable)
{
  rclcpp::init(0, nullptr);

  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");

  const auto declared = loader.getDeclaredClasses();
  ASSERT_THAT(declared,
    ::testing::Contains("wheel_smc_controller/WheelSmcController"))
    << "Plugin not registered. Check wheel_smc_controller.xml and "
       "PLUGINLIB_EXPORT_CLASS macro.";

  // Actually instantiate it to confirm the .so loads cleanly.
  auto instance = loader.createSharedInstance(
    "wheel_smc_controller/WheelSmcController");
  ASSERT_NE(instance, nullptr);

  rclcpp::shutdown();
}