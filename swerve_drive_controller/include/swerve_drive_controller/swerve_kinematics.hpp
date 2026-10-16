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

// Kinematics of a four-wheel independent steering, four-wheel independent
// drive (4WIS4WID) base. Kept free of ROS types so it can be unit tested.

#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_KINEMATICS_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_KINEMATICS_HPP_

#include <array>
#include <cstddef>

#include <Eigen/Dense>

namespace swerve_drive_controller
{

constexpr std::size_t NUM_WHEELS = 4;

/// Wheel order used everywhere in this package.
enum WheelIndex : std::size_t { FL = 0, BL = 1, BR = 2, FR = 3 };

/// Steering axis position in the base frame and wheel radius.
struct WheelGeometry
{
  double x{0.0};       // [m]
  double y{0.0};       // [m]
  double radius{0.0};  // [m]
};

using Geometry = std::array<WheelGeometry, NUM_WHEELS>;

/// Body twist expressed in the base frame.
struct BodyTwist
{
  double vx{0.0};  // [m/s]
  double vy{0.0};  // [m/s]
  double wz{0.0};  // [rad/s]
};

/// Velocity of the wheel contact point, as a heading and a signed speed.
struct WheelState
{
  double angle{0.0};  // steering angle [rad]
  double speed{0.0};  // linear speed along the wheel heading [m/s]
};

/// Inverse kinematics of one wheel:
///   v_x,i = vx - y_i * wz,   v_y,i = vy + x_i * wz
///   speed = |v_i|,           angle = atan2(v_y,i, v_x,i)
WheelState wheel_inverse_kinematics(const WheelGeometry & wheel, const BodyTwist & twist);

/// Shift the angle by a multiple of 2*pi so it is within pi of reference.
/// Same physical heading and speed: atan2 jumping from +pi to -pi does not
/// turn the wheel a full revolution.
WheelState unwrap_steering(const WheelState & target, double reference);

/// Flip the wheel (angle -+ pi, reversed speed) only if its angle is outside
/// [min_angle, max_angle]. If the range is shorter than pi and neither
/// orientation fits, the one closest to the range is clamped into it.
WheelState flip_into_limits(const WheelState & state, double min_angle, double max_angle);

/// Least-squares forward kinematics.
///
/// Each wheel contributes two equations
///   v_i cos(delta_i) = vx - y_i wz
///   v_i sin(delta_i) = vy + x_i wz
/// stacked as b = A * [vx vy wz]^T. The solution is xi = (A^T A)^-1 A^T b.
/// A depends only on the geometry, so the 3x8 pseudo-inverse is computed once.
/// For a symmetric layout this reduces to
///   vx = mean(v_i cos delta_i), vy = mean(v_i sin delta_i),
///   wz = sum(-y_i v_i cos delta_i + x_i v_i sin delta_i) / sum(x_i^2 + y_i^2)
class ForwardKinematics
{
public:
  /// Returns false if the geometry is degenerate (all wheels on one point).
  bool set_geometry(const Geometry & geometry);

  /// wheel_speed is the linear speed (or displacement) of each wheel along its
  /// heading; the result has the same unit per second (or per step).
  BodyTwist solve(
    const std::array<double, NUM_WHEELS> & wheel_speed,
    const std::array<double, NUM_WHEELS> & steering_angle) const;

private:
  Eigen::Matrix<double, 3, 2 * NUM_WHEELS> pseudo_inverse_{
    Eigen::Matrix<double, 3, 2 * NUM_WHEELS>::Zero()};
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_KINEMATICS_HPP_
