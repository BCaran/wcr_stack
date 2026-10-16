// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Enrique Fernández
 * Adapted from diff_drive_controller for swerve_drive_controller.
 */

#ifndef SWERVE_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_
#define SWERVE_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_

#include <cmath>

namespace swerve_drive_controller
{

class SpeedLimiter
{
public:
  /**
   * \param [in] has_velocity_limits     if true, applies velocity limits
   * \param [in] has_acceleration_limits if true, applies acceleration limits
   * \param [in] has_jerk_limits         if true, applies jerk limits
   * \param [in] min_velocity Minimum velocity, usually <= 0. NaN means -max_velocity.
   * \param [in] max_velocity Maximum velocity, usually >= 0
   * \param [in] min_acceleration Minimum acceleration, usually <= 0. NaN means -max_acceleration.
   * \param [in] max_acceleration Maximum acceleration, usually >= 0
   * \param [in] min_jerk Minimum jerk, usually <= 0. NaN means -max_jerk.
   * \param [in] max_jerk Maximum jerk, usually >= 0
   * \throws std::runtime_error if a limit is enabled but its maximum is NaN.
   */
  SpeedLimiter(
    bool has_velocity_limits = false, bool has_acceleration_limits = false,
    bool has_jerk_limits = false, double min_velocity = NAN, double max_velocity = NAN,
    double min_acceleration = NAN, double max_acceleration = NAN, double min_jerk = NAN,
    double max_jerk = NAN);

  /**
   * \brief Limit the velocity, acceleration, and jerk
   * \param [in, out] v  Velocity
   * \param [in]      v0 Previous velocity to v
   * \param [in]      v1 Previous velocity to v0
   * \param [in]      dt Time step [s]
   * \return Limiting factor (1.0 if none)
   */
  double limit(double & v, double v0, double v1, double dt);

  double limit_velocity(double & v);
  double limit_acceleration(double & v, double v0, double dt);
  double limit_jerk(double & v, double v0, double v1, double dt);

private:
  bool has_velocity_limits_;
  bool has_acceleration_limits_;
  bool has_jerk_limits_;

  double min_velocity_;
  double max_velocity_;

  double min_acceleration_;
  double max_acceleration_;

  double min_jerk_;
  double max_jerk_;
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SPEED_LIMITER_HPP_
