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
#include <cmath>

#include "rclcpp/time.hpp"
#include "swerve_drive_controller/odometry.hpp"
#include "swerve_drive_controller/speed_limiter.hpp"
#include "swerve_drive_controller/swerve_kinematics.hpp"

using swerve_drive_controller::BodyTwist;
using swerve_drive_controller::ForwardKinematics;
using swerve_drive_controller::Geometry;
using swerve_drive_controller::NUM_WHEELS;
using swerve_drive_controller::Odometry;
using swerve_drive_controller::flip_into_limits;
using swerve_drive_controller::SpeedLimiter;
using swerve_drive_controller::unwrap_steering;
using swerve_drive_controller::wheel_inverse_kinematics;
using swerve_drive_controller::WheelState;

namespace
{
constexpr double EPS = 1e-9;
constexpr double A = 0.1125;
constexpr double R = 0.0254;

// WCR layout: fl, bl, br, fr
const Geometry SQUARE = {{{A, A, R}, {-A, A, R}, {-A, -A, R}, {A, -A, R}}};
// Off-centre, unequal layout as it could come out of a calibration.
const Geometry SKEWED = {{{0.13, 0.10, 0.025}, {-0.09, 0.12, 0.026}, {-0.11, -0.10, 0.024},
  {0.12, -0.13, 0.0255}}};

/// Pose after moving with a constant body twist for time t, starting at the origin.
std::array<double, 3> analytic_pose(const BodyTwist & twist, double t)
{
  const double th = twist.wz * t;
  if (std::abs(twist.wz) < 1e-12) {
    return {twist.vx * t, twist.vy * t, 0.0};
  }
  return {
    (twist.vx * std::sin(th) + twist.vy * (std::cos(th) - 1.0)) / twist.wz,
    (twist.vx * (1.0 - std::cos(th)) + twist.vy * std::sin(th)) / twist.wz, th};
}

/// Joint velocities [rad/s] and steering angles that realise a twist.
void joint_states_for(
  const Geometry & geometry, const BodyTwist & twist, std::array<double, NUM_WHEELS> & velocity,
  std::array<double, NUM_WHEELS> & angle)
{
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    const WheelState wheel = wheel_inverse_kinematics(geometry[i], twist);
    velocity[i] = wheel.speed / geometry[i].radius;
    angle[i] = wheel.angle;
  }
}
}  // namespace

// ------------------------------------------------------- inverse kinematics

TEST(SwerveKinematics, forward_motion_points_all_wheels_forward)
{
  for (const auto & wheel : SQUARE) {
    const WheelState state = wheel_inverse_kinematics(wheel, BodyTwist{0.2, 0.0, 0.0});
    EXPECT_NEAR(state.angle, 0.0, EPS);
    EXPECT_NEAR(state.speed, 0.2, EPS);
  }
}

TEST(SwerveKinematics, lateral_motion_points_all_wheels_left)
{
  for (const auto & wheel : SQUARE) {
    const WheelState state = wheel_inverse_kinematics(wheel, BodyTwist{0.0, 0.2, 0.0});
    EXPECT_NEAR(state.angle, M_PI_2, EPS);
    EXPECT_NEAR(state.speed, 0.2, EPS);
  }
}

TEST(SwerveKinematics, rotation_in_place_is_tangential)
{
  const double wz = 0.7;
  for (const auto & wheel : SQUARE) {
    const WheelState state = wheel_inverse_kinematics(wheel, BodyTwist{0.0, 0.0, wz});
    EXPECT_NEAR(state.speed, wz * std::hypot(wheel.x, wheel.y), EPS);
    // Tangent is perpendicular to the radius vector, counter-clockwise.
    EXPECT_NEAR(std::cos(state.angle) * wheel.x + std::sin(state.angle) * wheel.y, 0.0, EPS);
    EXPECT_NEAR(std::atan2(std::sin(state.angle), std::cos(state.angle)),
      std::atan2(wheel.x, -wheel.y), EPS);
  }
}

// --------------------------------------------------------- unwrap and flip

TEST(SwerveKinematics, unwrap_follows_reference_across_pi)
{
  // atan2 jumps from +pi to -pi, the steering reference must not.
  const WheelState out = unwrap_steering(WheelState{-3.0, 1.0}, 3.0);
  EXPECT_NEAR(out.angle, -3.0 + 2 * M_PI, EPS);
  EXPECT_NEAR(out.speed, 1.0, EPS);
}

TEST(SwerveKinematics, unwrap_keeps_multi_turn_reference)
{
  const WheelState out = unwrap_steering(WheelState{0.2, 1.0}, 4 * M_PI + 0.1);
  EXPECT_NEAR(out.angle, 4 * M_PI + 0.2, EPS);
  EXPECT_NEAR(out.speed, 1.0, EPS);
}

TEST(SwerveKinematics, no_flip_inside_limits)
{
  // Even when the flipped angle would be closer to the previous reference.
  for (const double angle : {-1.5, -0.4, 0.0, 0.7, 1.5}) {
    const WheelState out = flip_into_limits(WheelState{angle, 1.0}, -M_PI_2, M_PI_2);
    EXPECT_NEAR(out.angle, angle, EPS);
    EXPECT_NEAR(out.speed, 1.0, EPS);
  }
  const WheelState out = flip_into_limits(WheelState{0.3, 1.0}, -M_PI, M_PI);
  EXPECT_NEAR(out.angle, 0.3, EPS);
  EXPECT_NEAR(out.speed, 1.0, EPS);
}

TEST(SwerveKinematics, angle_on_limit_is_not_flipped)
{
  const WheelState out = flip_into_limits(WheelState{M_PI_2, 1.0}, -M_PI_2, M_PI_2);
  EXPECT_NEAR(out.angle, M_PI_2, EPS);
  EXPECT_NEAR(out.speed, 1.0, EPS);
}

TEST(SwerveKinematics, flips_when_above_max)
{
  const WheelState out = flip_into_limits(WheelState{3 * M_PI_4, 1.0}, -M_PI_2, M_PI_2);
  EXPECT_NEAR(out.angle, -M_PI_4, EPS);
  EXPECT_NEAR(out.speed, -1.0, EPS);
}

TEST(SwerveKinematics, flips_when_below_min)
{
  const WheelState out = flip_into_limits(WheelState{-M_PI_2 - 0.1, 0.5}, -M_PI_2, M_PI_2);
  EXPECT_NEAR(out.angle, M_PI_2 - 0.1, EPS);
  EXPECT_NEAR(out.speed, -0.5, EPS);
}

TEST(SwerveKinematics, wide_limits_flip_only_at_the_limit)
{
  const double max = 3 * M_PI_2;
  // Unwrapped past pi but still inside +-3pi/2: no flip.
  WheelState out = flip_into_limits(unwrap_steering(WheelState{-3.0, 1.0}, 3.0), -max, max);
  EXPECT_NEAR(out.angle, -3.0 + 2 * M_PI, EPS);
  EXPECT_NEAR(out.speed, 1.0, EPS);

  // Unwrapped past 3pi/2: flipped back by pi.
  out = flip_into_limits(unwrap_steering(WheelState{-1.5, 1.0}, 4.5), -max, max);
  EXPECT_NEAR(out.angle, -1.5 + M_PI, EPS);
  EXPECT_NEAR(out.speed, -1.0, EPS);
}

TEST(SwerveKinematics, limits_changed_at_runtime_bring_wheel_back_into_range)
{
  // Wheel was turned to 2pi with limits off, limits are now +-pi/2.
  const WheelState out = flip_into_limits(WheelState{2 * M_PI + 0.2, 1.0}, -M_PI_2, M_PI_2);
  EXPECT_NEAR(out.angle, 0.2, EPS);
  EXPECT_NEAR(out.speed, 1.0, EPS);
}

TEST(SwerveKinematics, short_range_clamps_to_nearest_limit)
{
  WheelState out = flip_into_limits(WheelState{1.2, 1.0}, -1.0, 1.0);
  EXPECT_NEAR(out.angle, 1.0, EPS);
  EXPECT_NEAR(out.speed, 1.0, EPS);

  out = flip_into_limits(WheelState{2.5, 1.0}, -1.0, 1.0);
  EXPECT_NEAR(out.angle, 2.5 - M_PI, EPS);
  EXPECT_NEAR(out.speed, -1.0, EPS);
}

// ------------------------------------------------------ forward kinematics

TEST(SwerveKinematics, forward_kinematics_inverts_inverse_kinematics)
{
  const BodyTwist twists[] = {
    {0.3, 0.0, 0.0}, {0.0, -0.2, 0.0}, {0.0, 0.0, 1.1}, {0.15, -0.07, 0.4}, {-0.2, 0.1, -0.9}};
  for (const auto & geometry : {SQUARE, SKEWED}) {
    ForwardKinematics fk;
    ASSERT_TRUE(fk.set_geometry(geometry));
    for (const auto & twist : twists) {
      std::array<double, NUM_WHEELS> speed{};
      std::array<double, NUM_WHEELS> angle{};
      for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        // Use the flipped representation on some wheels: same physical motion.
        WheelState wheel = wheel_inverse_kinematics(geometry[i], twist);
        if (i % 2 == 1) {
          wheel = WheelState{wheel.angle - M_PI, -wheel.speed};
        }
        speed[i] = wheel.speed;
        angle[i] = wheel.angle;
      }
      const BodyTwist out = fk.solve(speed, angle);
      EXPECT_NEAR(out.vx, twist.vx, 1e-12);
      EXPECT_NEAR(out.vy, twist.vy, 1e-12);
      EXPECT_NEAR(out.wz, twist.wz, 1e-12);
    }
  }
}

TEST(SwerveKinematics, matches_wcr_odometry_formula_for_square_layout)
{
  // wcr_odometry: vx = sum(cos d_i v_i)/4, vy = sum(sin d_i v_i)/4,
  // wz = sum((-y_i cos d_i + x_i sin d_i) v_i / (4 (x_i^2 + y_i^2)))
  ForwardKinematics fk;
  ASSERT_TRUE(fk.set_geometry(SQUARE));
  const std::array<double, NUM_WHEELS> speed = {0.11, 0.09, 0.13, 0.08};
  const std::array<double, NUM_WHEELS> angle = {0.2, -0.4, 0.9, 0.1};
  double vx = 0.0, vy = 0.0, wz = 0.0;
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    vx += std::cos(angle[i]) * speed[i] / 4.0;
    vy += std::sin(angle[i]) * speed[i] / 4.0;
    const double x = SQUARE[i].x, y = SQUARE[i].y;
    wz += (-y * std::cos(angle[i]) + x * std::sin(angle[i])) * speed[i] /
      (4.0 * (x * x + y * y));
  }
  const BodyTwist out = fk.solve(speed, angle);
  EXPECT_NEAR(out.vx, vx, 1e-12);
  EXPECT_NEAR(out.vy, vy, 1e-12);
  EXPECT_NEAR(out.wz, wz, 1e-12);
}

TEST(SwerveKinematics, rejects_degenerate_geometry)
{
  ForwardKinematics fk;
  const Geometry all_at_origin = {{{0, 0, R}, {0, 0, R}, {0, 0, R}, {0, 0, R}}};
  EXPECT_FALSE(fk.set_geometry(all_at_origin));
}

// ---------------------------------------------------------------- odometry

class SwerveOdometryTest : public ::testing::TestWithParam<BodyTwist>
{
};

TEST_P(SwerveOdometryTest, velocity_feedback_matches_analytic_pose)
{
  const BodyTwist twist = GetParam();
  Odometry odometry(1);
  ASSERT_TRUE(odometry.set_wheel_params(SKEWED));
  odometry.init(rclcpp::Time(0, 0));

  std::array<double, NUM_WHEELS> velocity{};
  std::array<double, NUM_WHEELS> angle{};
  joint_states_for(SKEWED, twist, velocity, angle);

  const double dt = 0.01;
  const int steps = 300;
  for (int k = 1; k <= steps; ++k) {
    ASSERT_TRUE(odometry.update_from_velocity(velocity, angle, rclcpp::Time(0, k * 10000000)));
  }

  const auto pose = analytic_pose(twist, steps * dt);
  EXPECT_NEAR(odometry.get_x(), pose[0], 1e-9);
  EXPECT_NEAR(odometry.get_y(), pose[1], 1e-9);
  EXPECT_NEAR(odometry.get_heading(), pose[2], 1e-9);
  EXPECT_NEAR(odometry.get_linear_x(), twist.vx, 1e-9);
  EXPECT_NEAR(odometry.get_linear_y(), twist.vy, 1e-9);
  EXPECT_NEAR(odometry.get_angular(), twist.wz, 1e-9);
}

TEST_P(SwerveOdometryTest, position_feedback_matches_analytic_pose)
{
  const BodyTwist twist = GetParam();
  Odometry odometry(1);
  ASSERT_TRUE(odometry.set_wheel_params(SKEWED));
  odometry.init(rclcpp::Time(0, 0));

  std::array<double, NUM_WHEELS> velocity{};
  std::array<double, NUM_WHEELS> angle{};
  joint_states_for(SKEWED, twist, velocity, angle);

  const double dt = 0.01;
  const int steps = 300;
  std::array<double, NUM_WHEELS> position = {1.0, -2.0, 0.5, 3.0};  // arbitrary start
  EXPECT_FALSE(odometry.update_from_position(position, angle, rclcpp::Time(0, 0)));
  for (int k = 1; k <= steps; ++k) {
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
      position[i] += velocity[i] * dt;
    }
    ASSERT_TRUE(odometry.update_from_position(position, angle, rclcpp::Time(0, k * 10000000)));
  }

  const auto pose = analytic_pose(twist, steps * dt);
  EXPECT_NEAR(odometry.get_x(), pose[0], 1e-9);
  EXPECT_NEAR(odometry.get_y(), pose[1], 1e-9);
  EXPECT_NEAR(odometry.get_heading(), pose[2], 1e-9);
}

TEST_P(SwerveOdometryTest, open_loop_matches_analytic_pose)
{
  const BodyTwist twist = GetParam();
  Odometry odometry;
  odometry.init(rclcpp::Time(0, 0));
  for (int k = 1; k <= 300; ++k) {
    odometry.update_open_loop(twist, rclcpp::Time(0, k * 10000000));
  }
  const auto pose = analytic_pose(twist, 3.0);
  EXPECT_NEAR(odometry.get_x(), pose[0], 1e-9);
  EXPECT_NEAR(odometry.get_y(), pose[1], 1e-9);
  EXPECT_NEAR(odometry.get_heading(), pose[2], 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
  Twists, SwerveOdometryTest,
  ::testing::Values(
    BodyTwist{0.2, 0.0, 0.0}, BodyTwist{0.0, 0.15, 0.0}, BodyTwist{0.0, 0.0, 0.8},
    BodyTwist{0.1, 0.0, 0.5}, BodyTwist{0.12, -0.08, 0.6}, BodyTwist{-0.05, 0.1, -1.2}));

TEST(SwerveOdometry, reset_keeps_wheel_history)
{
  Odometry odometry(1);
  ASSERT_TRUE(odometry.set_wheel_params(SQUARE));
  odometry.init(rclcpp::Time(0, 0));
  const std::array<double, NUM_WHEELS> angle{};
  odometry.update_from_position({0, 0, 0, 0}, angle, rclcpp::Time(0, 0));
  odometry.update_from_position({1, 1, 1, 1}, angle, rclcpp::Time(1, 0));
  EXPECT_NEAR(odometry.get_x(), R, EPS);

  odometry.reset_odometry();
  EXPECT_EQ(odometry.get_x(), 0.0);
  odometry.update_from_position({2, 2, 2, 2}, angle, rclcpp::Time(2, 0));
  EXPECT_NEAR(odometry.get_x(), R, EPS);
}

// ----------------------------------------------------------- speed limiter

TEST(SpeedLimiter, limits_velocity_and_acceleration)
{
  SpeedLimiter limiter(true, true, false, NAN, 1.0, NAN, 2.0);
  double v = 5.0;
  limiter.limit(v, 0.0, 0.0, 0.1);
  EXPECT_NEAR(v, 0.2, EPS);  // acceleration: 2.0 * 0.1

  v = -5.0;
  limiter.limit(v, -0.95, -0.9, 0.1);
  EXPECT_NEAR(v, -1.0, EPS);  // velocity, min defaults to -max
}

TEST(SpeedLimiter, throws_without_max)
{
  EXPECT_THROW(SpeedLimiter(true, false, false), std::runtime_error);
}
