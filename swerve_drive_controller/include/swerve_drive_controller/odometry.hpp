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

// Wheel odometry for a 4WIS4WID base. Structure follows
// diff_drive_controller::Odometry; the forward kinematics are the
// least-squares solution from swerve_kinematics.hpp.

#ifndef SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <array>
#include <cstddef>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"
#include "swerve_drive_controller/swerve_kinematics.hpp"

namespace swerve_drive_controller
{

class Odometry
{
public:
  explicit Odometry(std::size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);

  /// Update from driving joint positions [rad] and steering angles [rad].
  /// The first call after init() only stores the wheel positions.
  bool update_from_position(
    const std::array<double, NUM_WHEELS> & wheel_position,
    const std::array<double, NUM_WHEELS> & steering_angle, const rclcpp::Time & time);

  /// Update from driving joint velocities [rad/s] and steering angles [rad].
  bool update_from_velocity(
    const std::array<double, NUM_WHEELS> & wheel_velocity,
    const std::array<double, NUM_WHEELS> & steering_angle, const rclcpp::Time & time);

  /// Integrate a commanded body twist.
  void update_open_loop(const BodyTwist & twist, const rclcpp::Time & time);

  void reset_odometry();

  double get_x() const { return x_; }
  double get_y() const { return y_; }
  double get_heading() const { return heading_; }
  double get_linear_x() const { return linear_x_; }
  double get_linear_y() const { return linear_y_; }
  double get_angular() const { return angular_; }

  /// Returns false if the geometry is degenerate.
  bool set_wheel_params(const Geometry & geometry);
  void set_velocity_rolling_window_size(std::size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  /// Body-frame displacement over one step.
  void integrate(double dx, double dy, double dtheta);
  void integrate_runge_kutta2(double dx, double dy, double dtheta);
  void integrate_exact(double dx, double dy, double dtheta);
  void accumulate_velocity(const BodyTwist & displacement, double dt);
  void reset_accumulators();

  rclcpp::Time timestamp_;

  double x_{0.0};        // [m]
  double y_{0.0};        // [m]
  double heading_{0.0};  // [rad]

  double linear_x_{0.0};  // [m/s], base frame
  double linear_y_{0.0};  // [m/s], base frame
  double angular_{0.0};   // [rad/s]

  Geometry geometry_{};
  ForwardKinematics forward_kinematics_;

  std::array<double, NUM_WHEELS> wheel_old_position_{};
  std::array<double, NUM_WHEELS> steering_old_angle_{};
  bool has_old_position_{false};

  std::size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
