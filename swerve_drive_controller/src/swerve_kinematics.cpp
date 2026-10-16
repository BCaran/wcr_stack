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

#include "swerve_drive_controller/swerve_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace swerve_drive_controller
{

namespace
{
// Tolerance on the range check so angles exactly on a limit are accepted.
constexpr double ANGLE_EPSILON = 1e-9;
}  // namespace

WheelState wheel_inverse_kinematics(const WheelGeometry & wheel, const BodyTwist & twist)
{
  const double v_x = twist.vx - wheel.y * twist.wz;
  const double v_y = twist.vy + wheel.x * twist.wz;
  return WheelState{std::atan2(v_y, v_x), std::hypot(v_x, v_y)};
}

WheelState unwrap_steering(const WheelState & target, double reference)
{
  const double turns = std::round((reference - target.angle) / (2.0 * M_PI));
  return WheelState{target.angle + turns * 2.0 * M_PI, target.speed};
}

WheelState flip_into_limits(const WheelState & state, double min_angle, double max_angle)
{
  // Shift by n*pi towards the range; every odd n reverses the speed.
  // In normal operation n is 0 or 1. Larger n only happens when the limits
  // are changed at runtime while the wheel is outside them.
  const auto shift = [](WheelState & s, double n) {
      s.angle += n * M_PI;
      if (std::fmod(std::abs(n), 2.0) > 0.5) {
        s.speed = -s.speed;
      }
    };

  WheelState out = state;
  if (out.angle > max_angle) {
    shift(out, -std::ceil((out.angle - max_angle) / M_PI - ANGLE_EPSILON));
  }
  if (out.angle < min_angle) {
    shift(out, std::ceil((min_angle - out.angle) / M_PI - ANGLE_EPSILON));
  }
  if (out.angle > max_angle + ANGLE_EPSILON) {
    // Range shorter than pi: out is above max, out - pi is below min.
    // Keep the orientation that needs less clamping.
    const double above = out.angle - max_angle;
    const double below = min_angle - (out.angle - M_PI);
    if (below < above) {
      shift(out, -1.0);
    }
  }
  out.angle = std::clamp(out.angle, min_angle, max_angle);
  return out;
}

bool ForwardKinematics::set_geometry(const Geometry & geometry)
{
  Eigen::Matrix<double, 2 * NUM_WHEELS, 3> a;
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    a.row(2 * i) << 1.0, 0.0, -geometry[i].y;
    a.row(2 * i + 1) << 0.0, 1.0, geometry[i].x;
  }
  const Eigen::Matrix3d ata = a.transpose() * a;
  Eigen::FullPivLU<Eigen::Matrix3d> lu(ata);
  if (!lu.isInvertible()) {
    return false;
  }
  pseudo_inverse_ = lu.inverse() * a.transpose();
  return true;
}

BodyTwist ForwardKinematics::solve(
  const std::array<double, NUM_WHEELS> & wheel_speed,
  const std::array<double, NUM_WHEELS> & steering_angle) const
{
  Eigen::Matrix<double, 2 * NUM_WHEELS, 1> b;
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    b(2 * i) = wheel_speed[i] * std::cos(steering_angle[i]);
    b(2 * i + 1) = wheel_speed[i] * std::sin(steering_angle[i]);
  }
  const Eigen::Vector3d xi = pseudo_inverse_ * b;
  return BodyTwist{xi(0), xi(1), xi(2)};
}

}  // namespace swerve_drive_controller
