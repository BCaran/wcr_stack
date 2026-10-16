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

#include "swerve_drive_controller/odometry.hpp"

#include <cmath>

namespace swerve_drive_controller
{

namespace
{
// Same threshold as diff_drive_controller: below this, dt is too small to
// estimate a velocity from.
constexpr double MIN_DT = 0.0001;
// Below this rotation per step, the exact integration is ill-conditioned.
constexpr double MIN_DTHETA = 1e-6;
}  // namespace

Odometry::Odometry(std::size_t velocity_rolling_window_size)
: timestamp_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  reset_accumulators();
  has_old_position_ = false;
  timestamp_ = time;
}

bool Odometry::update_from_position(
  const std::array<double, NUM_WHEELS> & wheel_position,
  const std::array<double, NUM_WHEELS> & steering_angle, const rclcpp::Time & time)
{
  if (!has_old_position_) {
    wheel_old_position_ = wheel_position;
    steering_old_angle_ = steering_angle;
    has_old_position_ = true;
    timestamp_ = time;
    return false;
  }

  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < MIN_DT) {
    return false;
  }

  // Wheel displacement along its heading. The heading is taken at the middle
  // of the step, which is exact for a steering angle that changes linearly.
  std::array<double, NUM_WHEELS> displacement{};
  std::array<double, NUM_WHEELS> mid_angle{};
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    displacement[i] = (wheel_position[i] - wheel_old_position_[i]) * geometry_[i].radius;
    mid_angle[i] = 0.5 * (steering_angle[i] + steering_old_angle_[i]);
  }
  wheel_old_position_ = wheel_position;
  steering_old_angle_ = steering_angle;

  const BodyTwist delta = forward_kinematics_.solve(displacement, mid_angle);
  integrate(delta.vx, delta.vy, delta.wz);
  accumulate_velocity(delta, dt);
  timestamp_ = time;
  return true;
}

bool Odometry::update_from_velocity(
  const std::array<double, NUM_WHEELS> & wheel_velocity,
  const std::array<double, NUM_WHEELS> & steering_angle, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < MIN_DT) {
    return false;
  }

  std::array<double, NUM_WHEELS> displacement{};
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    displacement[i] = wheel_velocity[i] * geometry_[i].radius * dt;
  }

  const BodyTwist delta = forward_kinematics_.solve(displacement, steering_angle);
  integrate(delta.vx, delta.vy, delta.wz);
  accumulate_velocity(delta, dt);
  timestamp_ = time;
  return true;
}

void Odometry::update_open_loop(const BodyTwist & twist, const rclcpp::Time & time)
{
  linear_x_ = twist.vx;
  linear_y_ = twist.vy;
  angular_ = twist.wz;

  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrate(twist.vx * dt, twist.vy * dt, twist.wz * dt);
}

void Odometry::reset_odometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

bool Odometry::set_wheel_params(const Geometry & geometry)
{
  if (!forward_kinematics_.set_geometry(geometry)) {
    return false;
  }
  geometry_ = geometry;
  return true;
}

void Odometry::set_velocity_rolling_window_size(std::size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  reset_accumulators();
}

void Odometry::integrate(double dx, double dy, double dtheta)
{
  if (std::fabs(dtheta) < MIN_DTHETA) {
    integrate_runge_kutta2(dx, dy, dtheta);
  } else {
    integrate_exact(dx, dy, dtheta);
  }
}

void Odometry::integrate_runge_kutta2(double dx, double dy, double dtheta)
{
  const double direction = heading_ + 0.5 * dtheta;
  x_ += dx * std::cos(direction) - dy * std::sin(direction);
  y_ += dx * std::sin(direction) + dy * std::cos(direction);
  heading_ += dtheta;
}

void Odometry::integrate_exact(double dx, double dy, double dtheta)
{
  // Constant body twist over the step, heading theta(t) = theta0 + w*t:
  //   x' = vx cos(theta) - vy sin(theta),  y' = vx sin(theta) + vy cos(theta)
  // Integrating with dx = vx*dt, dy = vy*dt, dtheta = w*dt gives the update below.
  const double heading_old = heading_;
  heading_ += dtheta;
  const double ds = std::sin(heading_) - std::sin(heading_old);
  const double dc = std::cos(heading_) - std::cos(heading_old);
  x_ += (dx * ds + dy * dc) / dtheta;
  y_ += (-dx * dc + dy * ds) / dtheta;
}

void Odometry::accumulate_velocity(const BodyTwist & displacement, double dt)
{
  linear_x_accumulator_.accumulate(displacement.vx / dt);
  linear_y_accumulator_.accumulate(displacement.vy / dt);
  angular_accumulator_.accumulate(displacement.wz / dt);

  linear_x_ = linear_x_accumulator_.getRollingMean();
  linear_y_ = linear_y_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();
}

void Odometry::reset_accumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace swerve_drive_controller
