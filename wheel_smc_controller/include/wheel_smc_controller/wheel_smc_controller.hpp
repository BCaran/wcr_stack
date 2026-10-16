// Copyright 2026
// MIMO sliding-mode controller for 4WIS4WID driving wheels.
// Reference: Lee & Li, "Kinematics, dynamics and control design of 4WIS4WID
// mobile robots", J. Eng. 2014. doi:10.1049/joe.2014.0241

#ifndef WHEEL_SMC_CONTROLLER__WHEEL_SMC_CONTROLLER_HPP_
#define WHEEL_SMC_CONTROLLER__WHEEL_SMC_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "wheel_smc_controller/visibility_control.h"

namespace wheel_smc_controller
{

/// Reference packet expected on the velocity-reference topic.
/// The Float64MultiArray .data field accepts 3 lengths:
///   4 -> [v1c..v4c]                           (no feedforward)
///   8 -> [v1c..v4c, v1c_dot..v4c_dot]         (recommended)
///   9 -> [v1c..v4c, v1c_dot..v4c_dot, theta_dot]  (theta_dot ignored for now)
///
/// The 8-element form is recommended: publish v_dot analytically from your
/// kinematic node (it falls out of paper eq. 18 in closed form) so the SMC
/// equivalent-control feedforward term works as intended.
struct VelocityReference
{
  Eigen::Vector4d v_rc{Eigen::Vector4d::Zero()};
  Eigen::Vector4d v_rc_dot{Eigen::Vector4d::Zero()};
  double theta_dot{0.0};
};

class WheelSmcController : public controller_interface::ControllerInterface
{
public:
  WHEEL_SMC_CONTROLLER_PUBLIC
  WheelSmcController();

  WHEEL_SMC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  WHEEL_SMC_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  WHEEL_SMC_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  WHEEL_SMC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  WHEEL_SMC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  WHEEL_SMC_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  WHEEL_SMC_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---------------- parameters ----------------
  std::vector<std::string> drive_joint_names_;   // size 4
  std::vector<std::string> steer_joint_names_;   // size 4 (state only)
  std::array<double, 4> x_r_w_{};                // wheel x positions, robot frame
  std::array<double, 4> y_r_w_{};                // wheel y positions, robot frame
  double m_{27.0};                               // total mass (mb + 4*mw)
  double I_{1.6};                                // total inertia about θ axis
  double Iw_{0.003};                             // wheel rolling inertia
  double r_{0.05};                               // wheel radius
  Eigen::Vector4d lambda_{Eigen::Vector4d::Constant(5.0)};   // sliding gain (integral)
  Eigen::Vector4d mu_{Eigen::Vector4d::Zero()};              // derivative damping
  Eigen::Vector4d k_switch_{Eigen::Vector4d::Constant(10.0)};// K = D̄ + η
  double eps_{0.05};                             // boundary layer
  double tau_max_{5.0};                          // wheel torque limit (Nm)
  double regularization_{1e-6};                  // Tikhonov on N̄_rr
  double e_dot_cutoff_hz_{20.0};                 // LPF cutoff for ė
  std::string vel_ref_topic_{"~/velocity_reference"};
  std::string odom_topic_{"odom"};
  bool use_odom_for_omega_{false};               // false → expect θ̇ in vel_ref msg
  bool disable_coriolis_{false};                 // true → drop A_r·v term

  // ---------------- runtime state ----------------
  Eigen::Vector4d integ_err_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d last_e_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d e_dot_filt_{Eigen::Vector4d::Zero()};
  std::array<double, 4> last_delta_{};
  bool first_update_{true};

  // Indices into command_interfaces_ / state_interfaces_ so we don't depend
  // on resource-manager ordering. Resolved in on_activate().
  std::array<size_t, 4> drive_cmd_idx_{};
  std::array<size_t, 4> drive_state_idx_{};
  std::array<size_t, 4> steer_state_idx_{};
  std::array<size_t, 4> steer_vel_idx_{};

  // ---------------- IO ----------------
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_ref_sub_;
  realtime_tools::RealtimeBuffer<VelocityReference> vel_ref_rt_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  realtime_tools::RealtimeBuffer<double> odom_omega_rt_;

  // Live parameter callback handle (keeps callback alive)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ---------------- helpers ----------------
  static inline double sat(double s, double e)
  {
    if (e <= 0.0) {
      return (s > 0.0) - (s < 0.0);  // sign()
    }
    if (std::abs(s) > e) {
      return (s > 0.0) ? 1.0 : -1.0;
    }
    return s / e;
  }

  /// Build the rolling-block matrices M̄_rr, V̄_m,rr, N̄_rr from page 4 of
  /// the paper, restricted to the 4 driving wheels.
  void build_dynamics(
    const std::array<double, 4> & delta,
    const std::array<double, 4> & dot_delta,
    double theta,
    double theta_dot,
    Eigen::Matrix4d & Mrr,
    Eigen::Matrix4d & Vrr,
    Eigen::Matrix4d & Nrr) const;

  controller_interface::CallbackReturn read_parameters();
};

}  // namespace wheel_smc_controller

#endif  // WHEEL_SMC_CONTROLLER__WHEEL_SMC_CONTROLLER_HPP_