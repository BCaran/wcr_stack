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

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "swerve_drive_controller/swerve_drive_controller.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::LoanedCommandInterface;
using hardware_interface::LoanedStateInterface;
using hardware_interface::StateInterface;
using lifecycle_msgs::msg::State;
using swerve_drive_controller::NUM_WHEELS;

namespace
{
constexpr double A = 0.1125;
constexpr double R = 0.0254;
constexpr double EPS = 1e-9;
const std::array<double, NUM_WHEELS> WHEEL_X = {A, -A, -A, A};
const std::array<double, NUM_WHEELS> WHEEL_Y = {A, A, -A, -A};
const std::array<std::string, NUM_WHEELS> DRIVE_JOINTS = {
  "FL_wheel", "BL_wheel", "BR_wheel", "FR_wheel"};
const std::array<std::string, NUM_WHEELS> STEERING_JOINTS = {
  "FL_steering", "BL_steering", "BR_steering", "FR_steering"};
}  // namespace

class TestableSwerveDriveController : public swerve_drive_controller::SwerveDriveController
{
public:
  using SwerveDriveController::SwerveDriveController;

  void set_command(double vx, double vy, double wz, const rclcpp::Time & stamp)
  {
    auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = stamp;
    msg->twist.linear.x = vx;
    msg->twist.linear.y = vy;
    msg->twist.angular.z = wz;
    received_velocity_msg_ptr_.writeFromNonRT(msg);
  }

  std::shared_ptr<geometry_msgs::msg::TwistStamped> last_received_command()
  {
    return *received_velocity_msg_ptr_.readFromNonRT();
  }

  const swerve_drive_controller::Odometry & odometry() const { return odometry_; }
};

class TestSwerveDriveController : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override { controller_ = std::make_unique<TestableSwerveDriveController>(); }

  void TearDown() override
  {
    // Loaned interfaces hold references into this fixture, destroy the
    // controller first.
    controller_.reset();
  }

  void init_controller(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    std::vector<rclcpp::Parameter> overrides;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
      const std::string suffix[] = {"fl", "bl", "br", "fr"};
      overrides.emplace_back("drive_joint_" + suffix[i], DRIVE_JOINTS[i]);
      overrides.emplace_back("steering_joint_" + suffix[i], STEERING_JOINTS[i]);
      overrides.emplace_back("x_w_" + suffix[i], WHEEL_X[i]);
      overrides.emplace_back("y_w_" + suffix[i], WHEEL_Y[i]);
      overrides.emplace_back("r_w_" + suffix[i], R);
    }
    // WCR steering limits; later entries in `parameters` override these.
    overrides.emplace_back("has_steering_limits", true);
    overrides.emplace_back("max_steering_angle", M_PI_2);
    overrides.insert(overrides.end(), parameters.begin(), parameters.end());
    auto node_options = rclcpp::NodeOptions().parameter_overrides(overrides);
    ASSERT_EQ(
      controller_->init(controller_name_, "", node_options),
      controller_interface::return_type::OK);
  }

  void assign_interfaces(const std::string & steering_command_type)
  {
    state_interfaces_.clear();
    command_interfaces_.clear();
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
      state_interfaces_.emplace_back(DRIVE_JOINTS[i], HW_IF_POSITION, &drive_position_[i]);
      state_interfaces_.emplace_back(DRIVE_JOINTS[i], HW_IF_VELOCITY, &drive_velocity_[i]);
      state_interfaces_.emplace_back(STEERING_JOINTS[i], HW_IF_POSITION, &steering_position_[i]);
      command_interfaces_.emplace_back(DRIVE_JOINTS[i], HW_IF_VELOCITY, &drive_command_[i]);
      command_interfaces_.emplace_back(
        STEERING_JOINTS[i], steering_command_type, &steering_command_[i]);
    }
    std::vector<LoanedStateInterface> loaned_states;
    for (auto & state : state_interfaces_) {
      loaned_states.emplace_back(state);
    }
    std::vector<LoanedCommandInterface> loaned_commands;
    for (auto & command : command_interfaces_) {
      loaned_commands.emplace_back(command);
    }
    controller_->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
  }

  void configure_and_activate(const std::string & steering_command_type = "position")
  {
    ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    assign_interfaces(steering_command_type);
    ASSERT_EQ(
      controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }

  controller_interface::return_type update(double dt = 0.01)
  {
    time_ += rclcpp::Duration::from_seconds(dt);
    return controller_->update(time_, rclcpp::Duration::from_seconds(dt));
  }

  /// Contact point velocity of wheel i realised by the current commands.
  std::array<double, 2> commanded_wheel_velocity(std::size_t i) const
  {
    const double v = drive_command_[i] * R;
    return {v * std::cos(steering_command_[i]), v * std::sin(steering_command_[i])};
  }

  void expect_commands_realise(double vx, double vy, double wz) const
  {
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
      const auto v = commanded_wheel_velocity(i);
      EXPECT_NEAR(v[0], vx - WHEEL_Y[i] * wz, 1e-9) << "wheel " << i;
      EXPECT_NEAR(v[1], vy + WHEEL_X[i] * wz, 1e-9) << "wheel " << i;
      EXPECT_GE(steering_command_[i], -M_PI_2 - EPS) << "wheel " << i;
      EXPECT_LE(steering_command_[i], M_PI_2 + EPS) << "wheel " << i;
    }
  }

  const std::string controller_name_ = "test_swerve_drive_controller";
  std::unique_ptr<TestableSwerveDriveController> controller_;
  rclcpp::Time time_{0, 0, RCL_ROS_TIME};

  std::array<double, NUM_WHEELS> drive_position_{};
  std::array<double, NUM_WHEELS> drive_velocity_{};
  std::array<double, NUM_WHEELS> steering_position_{};
  std::array<double, NUM_WHEELS> drive_command_{};
  std::array<double, NUM_WHEELS> steering_command_{};
  std::vector<StateInterface> state_interfaces_;
  std::vector<CommandInterface> command_interfaces_;
};

TEST_F(TestSwerveDriveController, init_fails_without_joint_names)
{
  auto node_options = rclcpp::NodeOptions();
  EXPECT_EQ(
    controller_->init(controller_name_, "", node_options), controller_interface::return_type::ERROR);
}

TEST_F(TestSwerveDriveController, interface_configuration_follows_steering_mode)
{
  init_controller({rclcpp::Parameter("steering_command_interface", "velocity")});
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);

  const auto command = controller_->command_interface_configuration();
  EXPECT_THAT(
    command.names, ::testing::ElementsAre(
      "FL_wheel/velocity", "BL_wheel/velocity", "BR_wheel/velocity", "FR_wheel/velocity",
      "FL_steering/velocity", "BL_steering/velocity", "BR_steering/velocity",
      "FR_steering/velocity"));
  const auto state = controller_->state_interface_configuration();
  EXPECT_THAT(
    state.names, ::testing::ElementsAre(
      "FL_wheel/velocity", "BL_wheel/velocity", "BR_wheel/velocity", "FR_wheel/velocity",
      "FL_steering/position", "BL_steering/position", "BR_steering/position",
      "FR_steering/position"));
}

TEST_F(TestSwerveDriveController, rejects_invalid_steering_mode)
{
  auto node_options = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("drive_joint_fl", "a"), rclcpp::Parameter("drive_joint_bl", "b"),
      rclcpp::Parameter("drive_joint_br", "c"), rclcpp::Parameter("drive_joint_fr", "d"),
      rclcpp::Parameter("steering_joint_fl", "e"), rclcpp::Parameter("steering_joint_bl", "f"),
      rclcpp::Parameter("steering_joint_br", "g"), rclcpp::Parameter("steering_joint_fr", "h"),
      rclcpp::Parameter("steering_command_interface", "effort")});
  EXPECT_EQ(
    controller_->init(controller_name_, "", node_options), controller_interface::return_type::ERROR);
}

TEST_F(TestSwerveDriveController, activate_fails_with_missing_interfaces)
{
  init_controller({rclcpp::Parameter("steering_command_interface", "velocity")});
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign_interfaces(HW_IF_POSITION);  // hardware offers position, controller wants velocity
  EXPECT_NE(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
}

TEST_F(TestSwerveDriveController, position_mode_forward)
{
  init_controller();
  configure_and_activate();

  controller_->set_command(0.1, 0.0, 0.0, time_);
  ASSERT_EQ(update(), controller_interface::return_type::OK);
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(drive_command_[i], 0.1 / R, EPS);
    EXPECT_NEAR(steering_command_[i], 0.0, EPS);
  }
}

TEST_F(TestSwerveDriveController, position_mode_realises_body_twist)
{
  init_controller();
  configure_and_activate();

  const std::array<std::array<double, 3>, 5> twists = {{
    {0.0, 0.1, 0.0}, {0.0, -0.1, 0.0}, {0.0, 0.0, 0.6}, {-0.08, 0.05, -0.4}, {0.1, 0.1, 0.0}}};
  for (const auto & twist : twists) {
    controller_->set_command(twist[0], twist[1], twist[2], time_);
    ASSERT_EQ(update(), controller_interface::return_type::OK);
    expect_commands_realise(twist[0], twist[1], twist[2]);
  }
}

/// Command a pure translation with the given heading.
void set_heading(TestableSwerveDriveController & controller, double heading, double speed,
  const rclcpp::Time & stamp)
{
  controller.set_command(speed * std::cos(heading), speed * std::sin(heading), 0.0, stamp);
}

TEST_F(TestSwerveDriveController, flips_only_when_steering_limit_is_exceeded)
{
  init_controller();
  configure_and_activate();

  set_heading(*controller_, M_PI_2 - 0.1, 0.1, time_);
  update();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(steering_command_[i], M_PI_2 - 0.1, EPS);
    EXPECT_GT(drive_command_[i], 0.0);
  }

  set_heading(*controller_, M_PI_2 + 0.1, 0.1, time_);
  update();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(steering_command_[i], -M_PI_2 + 0.1, EPS);
    EXPECT_NEAR(drive_command_[i], -0.1 / R, EPS);
  }
}

TEST_F(TestSwerveDriveController, min_steering_angle_defaults_to_minus_max)
{
  init_controller({rclcpp::Parameter("max_steering_angle", 2.0)});
  configure_and_activate();

  set_heading(*controller_, -1.9, 0.1, time_);  // inside [-2, 2]
  update();
  EXPECT_NEAR(steering_command_[0], -1.9, EPS);

  set_heading(*controller_, -2.1, 0.1, time_);  // below -2: flipped
  update();
  EXPECT_NEAR(steering_command_[0], -2.1 + M_PI, EPS);
  EXPECT_LT(drive_command_[0], 0.0);
}

TEST_F(TestSwerveDriveController, steering_limits_require_max)
{
  init_controller({rclcpp::Parameter("max_steering_angle", NAN)});
  EXPECT_NE(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);
}

TEST_F(TestSwerveDriveController, without_limits_steering_is_continuous)
{
  init_controller({rclcpp::Parameter("has_steering_limits", false)});
  configure_and_activate();

  // Heading turning through +-pi: the wheels keep turning the same way,
  // never flip and never unwind a full turn.
  for (const double heading : {2.9, 3.1, 3.3, 3.5, -2.6}) {
    set_heading(*controller_, heading, 0.1, time_);
    update();
    const double expected = heading < 0.0 ? heading + 2 * M_PI : heading;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
      EXPECT_NEAR(steering_command_[i], expected, EPS) << "heading " << heading;
      EXPECT_NEAR(drive_command_[i], 0.1 / R, EPS) << "heading " << heading;
    }
  }
}

TEST_F(TestSwerveDriveController, without_limits_velocity_loop_uses_shortest_error)
{
  init_controller(
    {rclcpp::Parameter("has_steering_limits", false),
      rclcpp::Parameter("steering_command_interface", "velocity"),
      rclcpp::Parameter("k_delta", 1.0),
      rclcpp::Parameter("steering_velocity_feedforward", false)});
  steering_position_ = {3.0, 3.0, 3.0, 3.0};
  configure_and_activate(HW_IF_VELOCITY);

  // Reference unwraps to 3.2, hardware reports the same angle wrapped.
  steering_position_.fill(3.15 - 2 * M_PI);
  set_heading(*controller_, 3.2 - 2 * M_PI, 0.1, time_);
  update();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(steering_command_[i], 0.05, 1e-9);
  }
}

TEST_F(TestSwerveDriveController, zero_command_holds_steering)
{
  init_controller();
  configure_and_activate();

  controller_->set_command(0.0, 0.0, 0.5, time_);
  update();
  const auto steering_before = steering_command_;

  controller_->set_command(0.0, 0.0, 0.0, time_);
  update();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(drive_command_[i], 0.0, EPS);
    EXPECT_NEAR(steering_command_[i], steering_before[i], EPS);
  }
}

TEST_F(TestSwerveDriveController, activation_starts_from_measured_steering)
{
  init_controller();
  steering_position_ = {0.3, -0.2, 0.1, 0.4};
  configure_and_activate();

  update();  // no command received yet
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(steering_command_[i], steering_position_[i], EPS);
    EXPECT_NEAR(drive_command_[i], 0.0, EPS);
  }
}

TEST_F(TestSwerveDriveController, stale_command_stops_the_robot)
{
  init_controller({rclcpp::Parameter("cmd_vel_timeout", 0.5)});
  configure_and_activate();

  controller_->set_command(0.1, 0.0, 0.0, time_);
  update();
  EXPECT_NEAR(drive_command_[0], 0.1 / R, EPS);

  update(0.6);
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(drive_command_[i], 0.0, EPS);
  }
}

TEST_F(TestSwerveDriveController, wheel_velocity_limit_keeps_direction)
{
  init_controller({rclcpp::Parameter("max_wheel_angular_velocity", 2.0)});
  configure_and_activate();

  controller_->set_command(0.1, 0.0, 0.6, time_);
  update();
  double max_velocity = 0.0;
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    max_velocity = std::max(max_velocity, std::abs(drive_command_[i]));
  }
  EXPECT_NEAR(max_velocity, 2.0, EPS);

  // Same twist direction, scaled down.
  const double vx = 0.1, wz = 0.6;
  const double scale = std::hypot(vx + A * wz, A * wz) / R / 2.0;
  expect_commands_realise(vx / scale, 0.0, wz / scale);
}

TEST_F(TestSwerveDriveController, body_acceleration_limit)
{
  init_controller(
    {rclcpp::Parameter("linear.x.has_acceleration_limits", true),
      rclcpp::Parameter("linear.x.max_acceleration", 1.0)});
  configure_and_activate();

  controller_->set_command(0.5, 0.0, 0.0, time_);
  update(0.01);
  EXPECT_NEAR(drive_command_[0] * R, 0.01, EPS);
  update(0.01);
  EXPECT_NEAR(drive_command_[0] * R, 0.02, EPS);
}

TEST_F(TestSwerveDriveController, velocity_mode_p_control)
{
  init_controller(
    {rclcpp::Parameter("steering_command_interface", "velocity"),
      rclcpp::Parameter("k_delta", 4.0),
      rclcpp::Parameter("steering_velocity_feedforward", false)});
  configure_and_activate(HW_IF_VELOCITY);

  steering_position_ = {0.1, 0.1, 0.1, 0.1};
  controller_->set_command(0.1, 0.1, 0.0, time_);  // reference pi/4 on all wheels
  update();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(steering_command_[i], 4.0 * (M_PI_4 - 0.1), EPS);
    EXPECT_NEAR(drive_command_[i], std::hypot(0.1, 0.1) / R, EPS);
  }
}

TEST_F(TestSwerveDriveController, velocity_mode_saturates_steering_velocity)
{
  init_controller(
    {rclcpp::Parameter("steering_command_interface", "velocity"),
      rclcpp::Parameter("k_delta", 10.0),
      rclcpp::Parameter("max_steering_velocity", 1.5)});
  configure_and_activate(HW_IF_VELOCITY);

  controller_->set_command(0.1, 0.1, 0.0, time_);
  update();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(steering_command_[i], 1.5, EPS);
  }

  steering_position_ = {1.5, 1.5, 1.5, 1.5};
  update();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_LT(steering_command_[i], 0.0);  // past the reference, turn back
    EXPECT_GE(steering_command_[i], -1.5);
  }
}

TEST_F(TestSwerveDriveController, velocity_mode_feedforward_follows_reference_rate)
{
  const double cutoff = 5.0;
  init_controller(
    {rclcpp::Parameter("steering_command_interface", "velocity"),
      rclcpp::Parameter("k_delta", 0.0),
      rclcpp::Parameter("steering_feedforward_cutoff_frequency", cutoff)});
  configure_and_activate(HW_IF_VELOCITY);

  // Lateral command rotating at a constant rate: reference angle ramps.
  const double rate = 0.5;  // rad/s
  const double dt = 0.01;
  double angle = 0.2;
  controller_->set_command(0.1 * std::cos(angle), 0.1 * std::sin(angle), 0.0, time_);
  update(dt);  // first step jumps from 0 to 0.2
  // 200 steps end at 1.2 rad, before the reference flips at pi/2.
  for (int k = 0; k < 200; ++k) {
    angle += rate * dt;
    controller_->set_command(0.1 * std::cos(angle), 0.1 * std::sin(angle), 0.0, time_);
    update(dt);
  }
  // Low-pass filter has converged to the reference rate.
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(steering_command_[i], rate, 1e-6);
  }
}

TEST_F(TestSwerveDriveController, scale_drive_by_steering_error)
{
  init_controller({rclcpp::Parameter("scale_drive_by_steering_error", true)});
  configure_and_activate();

  steering_position_ = {0.0, 0.0, 0.0, 0.0};
  controller_->set_command(0.1, 0.1, 0.0, time_);
  update();
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_NEAR(steering_command_[i], M_PI_4, EPS);
    EXPECT_NEAR(drive_command_[i], std::hypot(0.1, 0.1) / R * std::cos(M_PI_4), EPS);
  }
}

TEST_F(TestSwerveDriveController, runtime_gain_change)
{
  init_controller(
    {rclcpp::Parameter("steering_command_interface", "velocity"),
      rclcpp::Parameter("k_delta", 1.0),
      rclcpp::Parameter("steering_velocity_feedforward", false)});
  configure_and_activate(HW_IF_VELOCITY);

  controller_->set_command(0.1, 0.1, 0.0, time_);
  update();
  EXPECT_NEAR(steering_command_[0], M_PI_4, EPS);

  ASSERT_TRUE(controller_->get_node()->set_parameter(rclcpp::Parameter("k_delta", 3.0)).successful);
  update();
  EXPECT_NEAR(steering_command_[0], 3.0 * M_PI_4, EPS);
}

TEST_F(TestSwerveDriveController, runtime_geometry_change)
{
  init_controller();
  configure_and_activate();

  controller_->set_command(0.1, 0.0, 0.0, time_);
  update();
  EXPECT_NEAR(drive_command_[0], 0.1 / R, EPS);

  ASSERT_TRUE(controller_->get_node()->set_parameter(rclcpp::Parameter("r_w_fl", 0.03)).successful);
  update();
  EXPECT_NEAR(drive_command_[0], 0.1 / 0.03, EPS);
  EXPECT_NEAR(drive_command_[1], 0.1 / R, EPS);
}

TEST_F(TestSwerveDriveController, read_only_parameter_is_rejected_at_runtime)
{
  init_controller();
  configure_and_activate();
  EXPECT_FALSE(
    controller_->get_node()
    ->set_parameter(rclcpp::Parameter("steering_command_interface", "velocity"))
    .successful);
}

TEST_F(TestSwerveDriveController, odometry_from_velocity_feedback)
{
  init_controller({rclcpp::Parameter("velocity_rolling_window_size", 1)});
  configure_and_activate();

  // Wheels pointing sideways, all at 1 rad/s: pure lateral motion.
  steering_position_ = {M_PI_2, M_PI_2, M_PI_2, M_PI_2};
  drive_velocity_ = {1.0, 1.0, 1.0, 1.0};
  update();  // initialises odometry
  for (int k = 0; k < 100; ++k) {
    update(0.01);
  }
  EXPECT_NEAR(controller_->odometry().get_x(), 0.0, 1e-9);
  EXPECT_NEAR(controller_->odometry().get_y(), R * 1.0, 1e-9);
  EXPECT_NEAR(controller_->odometry().get_heading(), 0.0, 1e-9);
  EXPECT_NEAR(controller_->odometry().get_linear_y(), R, 1e-9);
}

TEST_F(TestSwerveDriveController, odometry_from_position_feedback)
{
  init_controller({rclcpp::Parameter("position_feedback", true)});
  configure_and_activate();

  // Rotation in place: wheels tangential, all rolling forward by 1 rad.
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    steering_position_[i] = std::atan2(WHEEL_X[i], -WHEEL_Y[i]);
  }
  // Keep the steering inside +-pi/2 by flipping, so the wheel turns backwards.
  std::array<double, NUM_WHEELS> direction{};
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    direction[i] = 1.0;
    if (steering_position_[i] > M_PI_2) {
      steering_position_[i] -= M_PI;
      direction[i] = -1.0;
    } else if (steering_position_[i] < -M_PI_2) {
      steering_position_[i] += M_PI;
      direction[i] = -1.0;
    }
  }
  update();  // stores initial wheel positions
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    drive_position_[i] += direction[i] * 1.0;
  }
  update();
  const double expected_heading = R / std::hypot(A, A);
  EXPECT_NEAR(controller_->odometry().get_heading(), expected_heading, 1e-9);
  EXPECT_NEAR(controller_->odometry().get_x(), 0.0, 1e-9);
  EXPECT_NEAR(controller_->odometry().get_y(), 0.0, 1e-9);
}

TEST_F(TestSwerveDriveController, odometry_open_loop)
{
  init_controller({rclcpp::Parameter("open_loop", true)});
  configure_and_activate();

  update();  // initialises odometry
  for (int k = 0; k < 100; ++k) {
    controller_->set_command(0.2, -0.1, 0.0, time_);
    update(0.01);
  }
  EXPECT_NEAR(controller_->odometry().get_x(), 0.2, 1e-9);
  EXPECT_NEAR(controller_->odometry().get_y(), -0.1, 1e-9);
}

TEST_F(TestSwerveDriveController, invalid_feedback_returns_error)
{
  init_controller();
  configure_and_activate();
  drive_velocity_[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(update(), controller_interface::return_type::ERROR);
}

TEST_F(TestSwerveDriveController, deactivate_stops_wheels)
{
  init_controller({rclcpp::Parameter("steering_command_interface", "velocity")});
  configure_and_activate(HW_IF_VELOCITY);

  controller_->set_command(0.1, 0.1, 0.0, time_);
  update();
  ASSERT_NE(drive_command_[0], 0.0);
  ASSERT_NE(steering_command_[0], 0.0);

  ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    EXPECT_EQ(drive_command_[i], 0.0);
    EXPECT_EQ(steering_command_[i], 0.0);
  }
}

TEST_F(TestSwerveDriveController, receives_twist_and_resets_odometry_over_ros)
{
  init_controller({rclcpp::Parameter("open_loop", true)});
  configure_and_activate();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  auto client_node = std::make_shared<rclcpp::Node>("swerve_test_client");
  executor.add_node(client_node);

  auto publisher = client_node->create_publisher<geometry_msgs::msg::TwistStamped>(
    controller_name_ + "/cmd_vel", rclcpp::SystemDefaultsQoS());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0u);

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = time_;
  msg.twist.linear.y = 0.05;
  publisher->publish(msg);
  for (int k = 0; k < 50 && controller_->last_received_command() == nullptr; ++k) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_NE(controller_->last_received_command(), nullptr);
  EXPECT_NEAR(controller_->last_received_command()->twist.linear.y, 0.05, EPS);

  update();
  update(0.1);
  EXPECT_GT(controller_->odometry().get_y(), 0.0);

  auto client =
    client_node->create_client<std_srvs::srv::Trigger>(controller_name_ + "/reset_odometry");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds(2)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);

  controller_->set_command(0.0, 0.0, 0.0, time_);
  update();
  EXPECT_NEAR(controller_->odometry().get_y(), 0.0, EPS);
}
