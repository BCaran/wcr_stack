# swerve_drive_controller

ros2_control controller for a four-wheel independent steering, four-wheel independent drive (4WIS4WID / swerve) base. It takes a body twist and writes driving wheel velocities and steering commands. It also publishes wheel odometry and the `odom → base_link` transform. Topics and parameters follow `diff_drive_controller`.

## Interfaces

| | Joint | Interface |
|---|---|---|
| Command | drive joints | `velocity` (always) |
| Command | steering joints | `position` or `velocity` (`steering_command_interface`) |
| State | drive joints | `velocity`, or `position` with `position_feedback: true` |
| State | steering joints | `position` |

| Topic / service | Type | |
|---|---|---|
| `~/cmd_vel` | `geometry_msgs/TwistStamped` | input, `use_stamped_vel: true` |
| `~/cmd_vel_unstamped` | `geometry_msgs/Twist` | input, `use_stamped_vel: false` |
| `~/cmd_vel_out` | `geometry_msgs/TwistStamped` | limited twist, `publish_limited_velocity: true` |
| `~/odom` | `nav_msgs/Odometry` | pose in `odom`, twist in `base_link` |
| `/tf` | `tf2_msgs/TFMessage` | `enable_odom_tf: true` |
| `~/reset_odometry` | `std_srvs/Trigger` | sets the pose back to zero |

## Control law

Wheels are indexed `fl, bl, br, fr`. Wheel *i* sits at (x_i, y_i) in `base_link` and has radius r_i.

1. **Body limits.** The twist (v_x, v_y, ω) goes through velocity, acceleration and jerk limits per axis, as in `diff_drive_controller`.
2. **Inverse kinematics.** v_xi = v_x − y_i ω and v_yi = v_y + x_i ω, so δ_i = atan2(v_yi, v_xi) and ω_i = |v_i| / r_i. When a wheel's speed drops below 0.1 mm/s, it keeps its previous steering angle instead of snapping to 0.
3. **Unwrap.** δ_i is shifted by 2π so that it stays within π of the previous reference. The wheel heading and speed don't change, but crossing ±π no longer turns the wheel a full revolution.
4. **Wheel flip.** This step only applies with `has_steering_limits: true`. If δ_i is outside `[min_steering_angle, max_steering_angle]`, the wheel is flipped: δ_i ∓ π, with the speed reversed. Inside the limits it is never flipped. `max_steering_angle` is required, and `min_steering_angle` defaults to `-max`. Without limits the steering is continuous. The velocity loop then uses the shortest angle error, so hardware that reports a wrapped angle still works.
5. **Wheel speed limit.** If any |ω_i| exceeds `max_wheel_angular_velocity`, all wheels are scaled down by the same factor, so the direction of motion is kept.
6. **Steering command.**
   - `position`: δ_i goes straight to the hardware.
   - `velocity`: δ̇_cmd = ff + `k_delta` · (δ_i − δ_meas), saturated to `max_steering_velocity`. The feedforward ff is the rate of change of the steering reference, dδ_i/dt, so the wheel turns along with a turning reference instead of always lagging behind it. That derivative comes from a finite difference each cycle, which is noisy: cmd_vel usually arrives slower than the 100 Hz loop, so the reference changes in steps. It is therefore smoothed by a first-order low-pass filter, y ← α·y + (1 − α)·x with α = exp(−2π·f_c·dt), where f_c is `steering_feedforward_cutoff_frequency` (`Fc_dot` in `model_based_controller_velocity`). A higher f_c reacts faster but lets more noise through; 5 Hz is a time constant of about 32 ms. The feedforward is set to zero on the step where a wheel flips.
7. **Optional.** With `scale_drive_by_steering_error: true`, ω_i is multiplied by max(0, cos(δ_i − δ_meas)).

## Odometry

The body twist is the least-squares solution of the eight wheel equations

    v_i cos δ_i = v_x − y_i ω
    v_i sin δ_i = v_y + x_i ω

The 3×8 pseudo-inverse depends only on the geometry and is recomputed when `x_w_*`, `y_w_*` or `r_w_*` change. For a symmetric layout it matches `wcr_odometry`. It also stays correct after calibration makes the layout asymmetric. The pose is integrated exactly for a constant body twist over each step, with RK2 for |Δθ| < 1e-6. Velocities are averaged over `velocity_rolling_window_size` samples.

## Parameters

All parameters are documented in [src/swerve_drive_controller_parameters.yaml](src/swerve_drive_controller_parameters.yaml), and [config/swerve_drive_controller.yaml](config/swerve_drive_controller.yaml) is a commented WCR configuration. Parameters that are not `read_only` can be changed while the controller runs. This includes the geometry, `k_delta`, the limits and the timeout:

```bash
ros2 param set /swerve_drive_controller k_delta 8.0
ros2 param set /swerve_drive_controller r_w_fl 0.0251
```

## Usage

```bash
ros2 launch wcr_bringup wcr_swerve.launch.py                          # position steering
ros2 launch wcr_bringup wcr_swerve.launch.py steering_mode:=velocity  # velocity steering
ros2 launch wcr_bringup mock_wcr_swerve.launch.py                     # no hardware, with RViz
```

Both launch files remap the controller to `/wcr/cmd_vel` (Twist), `/wcr/odom` and `/reset_odometry`. They replace `inv_kin_controller` + `wcr_odometry` without changes to teleop or the higher-level controllers.

## Tests

```bash
colcon build --packages-select swerve_drive_controller
colcon test --packages-select swerve_drive_controller --ctest-args -R swerve
colcon test-result --verbose
```

- `test_swerve_kinematics` covers inverse/forward kinematics, the wheel flip, odometry against closed-form poses, and the speed limiter.
- `test_swerve_drive_controller` runs the controller with fake hardware interfaces in both steering modes.
- `test_load_swerve_drive_controller` loads and configures the controller through `controller_manager`.
