// Copyright 2026
// MIMO sliding-mode controller for 4WIS4WID driving wheels.
// Reference: Lee & Li, J. Eng. 2014, doi:10.1049/joe.2014.0241

#include "wheel_smc_controller/wheel_smc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace wheel_smc_controller
{

WheelSmcController::WheelSmcController()
: controller_interface::ControllerInterface() {}

// ---------------------------------------------------------------------------
// on_init: declare parameters, default values
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn WheelSmcController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("drive_joints", {});
    auto_declare<std::vector<std::string>>("steer_joints", {});

    auto_declare<std::vector<double>>("wheel_x", {0.2, -0.2, -0.2, 0.2});
    auto_declare<std::vector<double>>("wheel_y", {0.25, 0.25, -0.25, -0.25});

    auto_declare<double>("mass_total", 27.0);
    auto_declare<double>("inertia_total", 1.6);
    auto_declare<double>("wheel_inertia", 0.003);
    auto_declare<double>("wheel_radius", 0.05);

    auto_declare<std::vector<double>>("lambda",   {5.0, 5.0, 5.0, 5.0});
    auto_declare<std::vector<double>>("mu",       {0.0, 0.0, 0.0, 0.0});
    auto_declare<std::vector<double>>("k_switch", {10.0, 10.0, 10.0, 10.0});
    auto_declare<double>("boundary_layer", 0.05);
    auto_declare<double>("torque_limit", 5.0);
    auto_declare<double>("regularization", 1e-6);
    auto_declare<double>("e_dot_cutoff_hz", 20.0);

    auto_declare<std::string>("velocity_reference_topic", "~/velocity_reference");
    auto_declare<std::string>("odom_topic", "odom");
    auto_declare<bool>("use_odom_for_omega", false);
    // If true, drop the A_r·v term from the SMC law. This term is the
    // model-based Coriolis-like feedback; if your steering velocity is
    // noisy or your dynamic parameters (m, I, Iφ) are uncertain, this
    // term injects spurious dynamics. Setting true reduces SMC to a
    // sliding-mode controller without the equivalent-control state
    // feedback — like a robust PI with model-based torque mapping.
    auto_declare<bool>("disable_coriolis", false);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read_parameters helper
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn WheelSmcController::read_parameters()
{
  auto node = get_node();

  drive_joint_names_ = node->get_parameter("drive_joints").as_string_array();
  steer_joint_names_ = node->get_parameter("steer_joints").as_string_array();

  if (drive_joint_names_.size() != 4 || steer_joint_names_.size() != 4) {
    RCLCPP_ERROR(node->get_logger(),
      "drive_joints and steer_joints must each have exactly 4 entries "
      "(got %zu and %zu)", drive_joint_names_.size(), steer_joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto wx = node->get_parameter("wheel_x").as_double_array();
  const auto wy = node->get_parameter("wheel_y").as_double_array();
  if (wx.size() != 4 || wy.size() != 4) {
    RCLCPP_ERROR(node->get_logger(), "wheel_x and wheel_y must have 4 entries");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < 4; ++i) {
    x_r_w_[i] = wx[i];
    y_r_w_[i] = wy[i];
  }

  m_  = node->get_parameter("mass_total").as_double();
  I_  = node->get_parameter("inertia_total").as_double();
  Iw_ = node->get_parameter("wheel_inertia").as_double();
  r_  = node->get_parameter("wheel_radius").as_double();

  if (r_ <= 0.0 || m_ <= 0.0 || I_ <= 0.0 || Iw_ <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Physical parameters must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto lam = node->get_parameter("lambda").as_double_array();
  const auto mu  = node->get_parameter("mu").as_double_array();
  const auto ks  = node->get_parameter("k_switch").as_double_array();
  if (lam.size() != 4 || mu.size() != 4 || ks.size() != 4) {
    RCLCPP_ERROR(node->get_logger(), "lambda, mu, k_switch must have 4 entries");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (int i = 0; i < 4; ++i) {
    lambda_(i)   = lam[i];
    mu_(i)       = mu[i];
    k_switch_(i) = ks[i];
  }

  eps_           = node->get_parameter("boundary_layer").as_double();
  tau_max_       = node->get_parameter("torque_limit").as_double();
  regularization_ = node->get_parameter("regularization").as_double();
  e_dot_cutoff_hz_ = node->get_parameter("e_dot_cutoff_hz").as_double();

  vel_ref_topic_      = node->get_parameter("velocity_reference_topic").as_string();
  odom_topic_         = node->get_parameter("odom_topic").as_string();
  use_odom_for_omega_ = node->get_parameter("use_odom_for_omega").as_bool();
  disable_coriolis_   = node->get_parameter("disable_coriolis").as_bool();

  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_configure: read params, set up subscribers
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn WheelSmcController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  auto node = get_node();

  // Velocity reference subscription. Accepts 3 message lengths:
  //   4 elements  -> [v1c, v2c, v3c, v4c]
  //                  v_dot is set to zero (slow-reference approximation;
  //                  feedforward term in SMC eq. 39 is dropped).
  //   8 elements  -> [v1c..v4c, v1c_dot..v4c_dot]
  //                  Recommended. Publish derivatives analytically from
  //                  your kinematic node — they fall out of eq. 18 in closed
  //                  form. This gives you the full SMC equivalent control.
  //   9 elements  -> [v1c..v4c, v1c_dot..v4c_dot, theta_dot]
  //                  theta_dot is currently unused in build_dynamics() (the
  //                  body heading cancels out of M̄_rr, V̄_m,rr, N̄_rr under a
  //                  common-shift argument). Field 8 is reserved for future
  //                  extensions; the controller accepts it but ignores it.
  vel_ref_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    vel_ref_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      const size_t n = msg->data.size();
      if (n != 4 && n != 8 && n != 9) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
          1000, "velocity_reference msg has %zu elements; expected 4, 8, or 9", n);
        return;
      }
      VelocityReference ref;
      for (int i = 0; i < 4; ++i) {
        ref.v_rc(i)     = msg->data[i];
        ref.v_rc_dot(i) = (n >= 8) ? msg->data[4 + i] : 0.0;
      }
      ref.theta_dot = (n >= 9) ? msg->data[8] : 0.0;
      vel_ref_rt_.writeFromNonRT(ref);
    });

  if (use_odom_for_omega_) {
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_omega_rt_.writeFromNonRT(msg->twist.twist.angular.z);
      });
    odom_omega_rt_.writeFromNonRT(0.0);
  }

  // Initial empty reference so update() doesn't read garbage on first cycles.
  vel_ref_rt_.writeFromNonRT(VelocityReference{});

  // Live parameter tuning: re-read tunable params whenever someone calls
  // `ros2 param set`. Only gains/limits are tunable — joint names, wheel
  // positions, and physics params would require re-resolving interfaces
  // and rebuilding dynamics, so they're locked at configure time.
  param_cb_handle_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params)
      -> rcl_interfaces::msg::SetParametersResult
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        const auto & name = p.get_name();
        try {
          if (name == "lambda") {
            const auto v = p.as_double_array();
            if (v.size() != 4) { result.successful = false; result.reason = "lambda needs 4"; return result; }
            for (int i = 0; i < 4; ++i) lambda_(i) = v[i];
          } else if (name == "mu") {
            const auto v = p.as_double_array();
            if (v.size() != 4) { result.successful = false; result.reason = "mu needs 4"; return result; }
            for (int i = 0; i < 4; ++i) mu_(i) = v[i];
          } else if (name == "k_switch") {
            const auto v = p.as_double_array();
            if (v.size() != 4) { result.successful = false; result.reason = "k_switch needs 4"; return result; }
            for (int i = 0; i < 4; ++i) k_switch_(i) = v[i];
          } else if (name == "boundary_layer") {
            eps_ = p.as_double();
          } else if (name == "torque_limit") {
            tau_max_ = p.as_double();
          } else if (name == "regularization") {
            regularization_ = p.as_double();
          } else if (name == "e_dot_cutoff_hz") {
            e_dot_cutoff_hz_ = p.as_double();
          } else if (name == "disable_coriolis") {
            disable_coriolis_ = p.as_bool();
          }
          // Silently ignore other params (e.g. joint names) — they're
          // not tunable at runtime; user must restart controller.
        } catch (const std::exception & e) {
          result.successful = false;
          result.reason = std::string("parameter update failed: ") + e.what();
          return result;
        }
      }
      return result;
    });

  RCLCPP_INFO(node->get_logger(), "WheelSmcController configured.");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface configurations
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
WheelSmcController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(4);
  for (const auto & j : drive_joint_names_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
WheelSmcController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(12);
  // Drive wheel velocities (rad/s)
  for (const auto & j : drive_joint_names_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  // Steering angles (rad)
  for (const auto & j : steer_joint_names_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  }
  // Steering velocities (rad/s) — read directly to avoid noisy finite-difference
  for (const auto & j : steer_joint_names_) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

// ---------------------------------------------------------------------------
// on_activate: resolve interface ordering, zero state
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn WheelSmcController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  // Resource manager may reorder interfaces. Resolve indices by name.
  auto find_cmd = [this](const std::string & full_name) -> long {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
      if (command_interfaces_[i].get_name() == full_name) {
        return static_cast<long>(i);
      }
    }
    return -1;
  };
  auto find_state = [this](const std::string & full_name) -> long {
    for (size_t i = 0; i < state_interfaces_.size(); ++i) {
      if (state_interfaces_[i].get_name() == full_name) {
        return static_cast<long>(i);
      }
    }
    return -1;
  };

  for (size_t i = 0; i < 4; ++i) {
    const std::string cmd  = drive_joint_names_[i] + "/" + hardware_interface::HW_IF_EFFORT;
    const std::string sv   = drive_joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY;
    const std::string ss   = steer_joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION;
    const std::string sdv  = steer_joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY;
    const long ci = find_cmd(cmd);
    const long si = find_state(sv);
    const long ti = find_state(ss);
    const long tvi = find_state(sdv);
    if (ci < 0 || si < 0 || ti < 0 || tvi < 0) {
      RCLCPP_ERROR(node->get_logger(),
        "Failed to resolve interface for drive=%s steer=%s",
        drive_joint_names_[i].c_str(), steer_joint_names_[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    drive_cmd_idx_[i]   = static_cast<size_t>(ci);
    drive_state_idx_[i] = static_cast<size_t>(si);
    steer_state_idx_[i] = static_cast<size_t>(ti);
    steer_vel_idx_[i]   = static_cast<size_t>(tvi);
  }

  integ_err_.setZero();
  last_e_.setZero();
  e_dot_filt_.setZero();
  for (size_t i = 0; i < 4; ++i) {
    last_delta_[i] = state_interfaces_[steer_state_idx_[i]].get_value();
  }
  first_update_ = true;

  // Set commands to zero on activation.
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[drive_cmd_idx_[i]].set_value(0.0);
  }

  RCLCPP_INFO(node->get_logger(), "WheelSmcController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate: zero out commands
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn WheelSmcController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[drive_cmd_idx_[i]].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// build_dynamics: M̄_rr, V̄_m,rr, N̄_rr (page 4 of Lee & Li 2014)
//
// Note: c_i = cos(δ_i + θ), s_i = sin(δ_i + θ). The full kinematic angle θ
// of the body cancels out of the required M̄_rr and N̄_rr entries when we
// look at c_i^2 + s_i^2 etc., but it does enter through the products
// c_j c_i + s_j s_i = cos(δ_i - δ_j), so θ itself does not appear; we only
// need δ_i and the body angular velocity θ̇ for V̄_m,rr.
// ---------------------------------------------------------------------------
void WheelSmcController::build_dynamics(
  const std::array<double, 4> & delta,
  const std::array<double, 4> & dot_delta,
  double /*theta*/,                     // not needed: only differences appear
  double theta_dot,
  Eigen::Matrix4d & Mrr,
  Eigen::Matrix4d & Vrr,
  Eigen::Matrix4d & Nrr) const
{
  // Precompute per-wheel quantities.
  std::array<double, 4> ci{}, si{}, dci{}, dsi{}, W{}, dW{};
  for (int i = 0; i < 4; ++i) {
    // Use δ_i alone: products c_j c_i + s_j s_i = cos(δ_i - δ_j) are
    // invariant under a common θ-shift. This avoids needing θ.
    const double a = delta[i];
    ci[i] = std::cos(a);
    si[i] = std::sin(a);

    const double xi = x_r_w_[i];
    const double yi = y_r_w_[i];
    const double denom = 4.0 * (xi * xi + yi * yi);
    W[i] = (-yi * ci[i] + xi * si[i]) / denom;

    // dδ_i/dt only (θ contribution drops with the same shift logic above)
    const double dd = dot_delta[i];
    dci[i] = -si[i] * dd;
    dsi[i] =  ci[i] * dd;
    dW[i]  = (-yi * dci[i] + xi * dsi[i]) / denom;
  }

  // Suppress unused warning if you re-enable θ later.
  (void)theta_dot;

  const double mass16 = m_ / 16.0;
  const double Iw_over_r2 = Iw_ / (r_ * r_);

  // --- M̄_rr ---  M_ii = I W_i^2 + m/16 (c_i^2+s_i^2) + Iw/r^2
  //               M_ij = I W_j W_i + m/16 (c_j c_i + s_j s_i)
  for (int i = 0; i < 4; ++i) {
    Mrr(i, i) = I_ * W[i] * W[i] + mass16 * (ci[i]*ci[i] + si[i]*si[i]) + Iw_over_r2;
    for (int j = 0; j < 4; ++j) {
      if (i == j) continue;
      Mrr(i, j) = I_ * W[j] * W[i] + mass16 * (ci[j]*ci[i] + si[j]*si[i]);
    }
  }

  // --- V̄_m,rr ---  V_ij = I W_j Ẇ_i + m/16 (c_j ċ_i + s_j ṡ_i)
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      Vrr(i, j) = I_ * W[j] * dW[i] + mass16 * (ci[j]*dci[i] + si[j]*dsi[i]);
    }
  }

  // --- N̄_rr ---  N_ii = W_i (y_i c(δ_i) + x_i s(δ_i))/r + (c_i^2+s_i^2)/(4r) + 1/r
  //               N_ij = W_i (y_j c(δ_j) + x_j s(δ_j))/r + (s_i s_j + c_i c_j)/(4r)
  // c_i in N is cos(δ_i + θ), but again only c_i c_j + s_i s_j enters → θ cancels.
  // The "y_i c(δ_i) + x_i s(δ_i)" term uses cos/sin of δ_i alone, per the paper.
  for (int i = 0; i < 4; ++i) {
    const double xi = x_r_w_[i];
    const double yi = y_r_w_[i];
    Nrr(i, i) = W[i] * (yi * ci[i] + xi * si[i]) / r_
              + (ci[i]*ci[i] + si[i]*si[i]) / (4.0 * r_)
              + 1.0 / r_;
    for (int j = 0; j < 4; ++j) {
      if (i == j) continue;
      const double xj = x_r_w_[j];
      const double yj = y_r_w_[j];
      Nrr(i, j) = W[i] * (yj * ci[j] + xj * si[j]) / r_
                + (si[i]*si[j] + ci[i]*ci[j]) / (4.0 * r_);
    }
  }

  // Tikhonov regularization on N̄_rr to keep B_r = M̄⁻¹ N̄ well-conditioned
  // when wheels are near-aligned (rows of N̄ become similar).
  if (regularization_ > 0.0) {
    for (int i = 0; i < 4; ++i) {
      Nrr(i, i) += regularization_;
    }
  }
}

// ---------------------------------------------------------------------------
// update: realtime control law
// ---------------------------------------------------------------------------
controller_interface::return_type WheelSmcController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= 0.0 || !std::isfinite(dt)) {
    return controller_interface::return_type::OK;
  }

  // ---- read state ----
  Eigen::Vector4d v_r;
  std::array<double, 4> delta{}, dot_delta{};
  for (size_t i = 0; i < 4; ++i) {
    const double omega_wheel = state_interfaces_[drive_state_idx_[i]].get_value();
    v_r(i) = omega_wheel * r_;                     // wheel linear velocity
    delta[i] = state_interfaces_[steer_state_idx_[i]].get_value();
    // Read steering velocity directly from hardware interface — finite-
    // differencing position at the controller rate produces noise spikes
    // that propagate through V_m → A_r → torque, causing low-frequency
    // limit-cycle oscillation in the wheel velocity.
    dot_delta[i] = state_interfaces_[steer_vel_idx_[i]].get_value();
    last_delta_[i] = delta[i];
  }

  const VelocityReference ref = *vel_ref_rt_.readFromRT();
  const double theta_dot = use_odom_for_omega_
      ? *odom_omega_rt_.readFromRT()
      : ref.theta_dot;

  first_update_ = false;

  // ---- error, integral, derivative, sliding surface ----
  // Paper's eq. 32 surface is S = e + λ·∫e dt (PI-like).
  // We extend to PID-like:  S = e + λ·∫e + μ·ė
  // where ė is approximated by low-pass-filtered finite difference.
  // μ acts like K_d in a PID — damps high-frequency excursions in e
  // without changing steady-state behavior.
  const Eigen::Vector4d e = ref.v_rc - v_r;
  integ_err_ += e * dt;

  // Filtered derivative of e (1st-order LPF on backward difference)
  Eigen::Vector4d e_dot;
  if (first_update_) {
    e_dot.setZero();
    e_dot_filt_ = e_dot;
  } else {
    const Eigen::Vector4d raw = (e - last_e_) / dt;
    // alpha = exp(-2π·fc·dt), with fc from boundary_layer-tied param
    const double alpha = std::exp(-2.0 * M_PI * e_dot_cutoff_hz_ * dt);
    e_dot_filt_ = alpha * e_dot_filt_ + (1.0 - alpha) * raw;
    e_dot = e_dot_filt_;
  }
  last_e_ = e;

  const Eigen::Vector4d S = e
                          + lambda_.asDiagonal() * integ_err_
                          + mu_.asDiagonal() * e_dot;

  // ---- build dynamics (page 4) ----
  Eigen::Matrix4d Mrr, Vrr, Nrr;
  build_dynamics(delta, dot_delta, /*theta=*/0.0, theta_dot, Mrr, Vrr, Nrr);

  // ---- A_r = M̄⁻¹ V̄_m, B_r = M̄⁻¹ N̄ ----
  Eigen::PartialPivLU<Eigen::Matrix4d> lu_M(Mrr);
  const Eigen::Matrix4d Ar = lu_M.solve(Vrr);
  const Eigen::Matrix4d Br = lu_M.solve(Nrr);

  // ---- SMC law (paper eq. 39, rolling block) ----
  Eigen::Vector4d sat_S;
  for (int i = 0; i < 4; ++i) {
    sat_S(i) = sat(S(i), eps_);
  }
  Eigen::Vector4d rhs = ref.v_rc_dot
                      + lambda_.asDiagonal() * e
                      + k_switch_.asDiagonal() * sat_S;
  if (!disable_coriolis_) {
    rhs += Ar * v_r;
  }

  Eigen::PartialPivLU<Eigen::Matrix4d> lu_B(Br);
  Eigen::Vector4d tau = lu_B.solve(rhs);

  if (!tau.allFinite()) {
    // Br near-singular or NaN somewhere: hold last command, log throttled.
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "SMC produced non-finite torque; holding zero this cycle.");
    tau.setZero();
  }

  // ---- saturate + conditional integral antiwindup ----
  for (int i = 0; i < 4; ++i) {
    if (std::abs(tau(i)) > tau_max_) {
      const double tau_sat = std::copysign(tau_max_, tau(i));
      // If the integrator is pushing further into saturation, undo this step.
      if (S(i) * e(i) > 0.0) {
        integ_err_(i) -= e(i) * dt;
      }
      tau(i) = tau_sat;
    }
  }

  // ---- write commands ----
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[drive_cmd_idx_[i]].set_value(tau(i));
  }

  return controller_interface::return_type::OK;
}

}  // namespace wheel_smc_controller

PLUGINLIB_EXPORT_CLASS(
  wheel_smc_controller::WheelSmcController,
  controller_interface::ControllerInterface)