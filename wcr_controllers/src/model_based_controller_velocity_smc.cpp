#include <cmath>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class ModelBasedControllerVelocity : public rclcpp::Node
{
public:
    ModelBasedControllerVelocity()
    : Node("trajectory_follower")
    {
        // sub
        desired_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wcr/desired_trajectory", 10,
            std::bind(&ModelBasedControllerVelocity::desired_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wcr/odom", 10,
            std::bind(&ModelBasedControllerVelocity::odom_callback, this, std::placeholders::_1));

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&ModelBasedControllerVelocity::joint_states_callback, this, std::placeholders::_1));

        // pub
        steering_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/steering_velocity_controller/commands", 10);

        // SMC velocity reference: Float64MultiArray with 8 elements:
        //   [v1c, v2c, v3c, v4c, v1c_dot, v2c_dot, v3c_dot, v4c_dot]
        // Wheel ordering matches the SMC controller's drive_joints param.
        smc_velocity_ref_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/smc_velocity_controller/velocity_reference", 10);

        output_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/wcr/output", 10);

        tracking_error_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/wcr/tracking_error", 10);

        // parameters
        this->declare_parameter("x_w_1", 0.1125);
        this->declare_parameter("y_w_1", 0.1125);
        this->declare_parameter("r_w_1", 0.0254);

        this->declare_parameter("x_w_2", -0.1125);
        this->declare_parameter("y_w_2", 0.1125);
        this->declare_parameter("r_w_2", 0.0254);

        this->declare_parameter("x_w_3", -0.1125);
        this->declare_parameter("y_w_3", -0.1125);
        this->declare_parameter("r_w_3", 0.0254);

        this->declare_parameter("x_w_4", 0.1125);
        this->declare_parameter("y_w_4", -0.1125);
        this->declare_parameter("r_w_4", 0.0254);

        this->declare_parameter("Kp_x", 5.0);
        this->declare_parameter("Kp_y", 5.0);
        this->declare_parameter("Kp_th", 5.0);
        this->declare_parameter("Kp_delta", 5.0);
        this->declare_parameter("Fc_dot", 5.0);
        this->declare_parameter("Fc_omega", 5.0);
        // LPF cutoff for v_dot reference (Hz). Set 0 to disable.
        this->declare_parameter("Fc_vdot", 20.0);

        RCLCPP_INFO(this->get_logger(), "ModelBasedControllerVelocity (SMC) node started");
    }

private:
    void desired_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(last_time_traj_ < 0){
          rclcpp::Time temp_time = msg->header.stamp;
          last_time_traj_ = temp_time.seconds();
          return;
        }
        rclcpp::Time temp_current_time = msg->header.stamp;
        double dt = temp_current_time.seconds() - last_time_traj_;
        last_time_traj_ = temp_current_time.seconds();
        if (dt <= 0.0) return;
        t_ += dt;

        double roll_d, pitch_d, yaw_d;
        quaternionToEuler(msg->pose.pose.orientation, roll_d, pitch_d, yaw_d);
        double x_d = msg->pose.pose.position.x;
        double y_d = msg->pose.pose.position.y;
        double theta_d = unwrap_d(yaw_d);

        geometry_msgs::msg::PoseStamped tracking_error_msg;
        tracking_error_msg.header.stamp = temp_current_time;
        tracking_error_msg.pose.position.x = x_d - x_;
        tracking_error_msg.pose.position.y = y_d - y_;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_d - theta_);
        q.normalize();
        tracking_error_msg.pose.orientation = tf2::toMsg(q);

        // velocities are received transformed in robot base_link frame
        double v_x_d = msg->twist.twist.linear.x;
        double v_y_d = msg->twist.twist.linear.y;
        double omega_d = msg->twist.twist.angular.z;

        // Error transformed in robot frame
        double e_x = (x_d - x_) * cos(theta_) + (y_d - y_) * sin(theta_);
        double e_y = -(x_d - x_) * sin(theta_) + (y_d - y_) * cos(theta_);
        double e_th = theta_d - theta_;

        // Error derivatives in robot frame (Kanayama-style; assumes
        // measured v_x_, v_y_ are in the robot frame as per odom_callback).
        // ė_x ≈ v_xd^r − v_x^r,  ė_y ≈ v_yd^r − v_y^r,  ė_θ = ω_d − ω
        const double e_x_dot  = v_x_d - v_x_;
        const double e_y_dot  = v_y_d - v_y_;
        const double e_th_dot = omega_d - omega_;

        double x_w[4] = {this->get_parameter("x_w_1").as_double(), this->get_parameter("x_w_2").as_double(),
                         this->get_parameter("x_w_3").as_double(), this->get_parameter("x_w_4").as_double()};
        double y_w[4] = {this->get_parameter("y_w_1").as_double(), this->get_parameter("y_w_2").as_double(),
                         this->get_parameter("y_w_3").as_double(), this->get_parameter("y_w_4").as_double()};
        double r_w[4] = {this->get_parameter("r_w_1").as_double(), this->get_parameter("r_w_2").as_double(),
                         this->get_parameter("r_w_3").as_double(), this->get_parameter("r_w_4").as_double()};
        Kp_x_ = this->get_parameter("Kp_x").as_double();
        Kp_y_ = this->get_parameter("Kp_y").as_double();
        Kp_th_ = this->get_parameter("Kp_th").as_double();
        Kp_delta_ = this->get_parameter("Kp_delta").as_double();

        double v_x_i[4], v_y_i[4], v_d_i[4], delta_d_i[4];
        double a[4], b[4];
        double v_lin_c_i[4];   // linear wheel velocity reference (m/s) for SMC
        double v_lin_dot_c_i[4]; // its time derivative (m/s²)
        double delta_c_i[4], dot_delta_c_i[4];
        const double max_angular_speed = (210*0.229) * ((2.0*M_PI)/60.0);
        double maxQuotien = 0.0;
        double sendingSteeringVelocity[4];

        // ---- One-time init of histories/filters
        if (!delta_hist_init_) {
          for (int i = 0; i < 4; ++i) {
            delta_hist_1_[i] = delta_i_[i];
            delta_hist_2_[i] = delta_i_[i];
          }
          delta_hist_init_ = true;
        }

        // ---- Main loop
        for (int i = 0; i < 4; ++i) {

          // --- Cartesian -> wheel-space velocities (open-loop / feedforward part)
          const double vx_ff = v_x_d - y_w[i] * omega_d;
          const double vy_ff = v_y_d + x_w[i] * omega_d;

          v_x_i[i]     = vx_ff;
          v_y_i[i]     = vy_ff;
          v_d_i[i]     = std::hypot(vx_ff, vy_ff);
          delta_d_i[i] = std::atan2(vy_ff, vx_ff);

          // --- Controller in wheel space (paper eq. 18 form)
          //   a_i = vx_ff + Kx*e_x  − Kθ*y_w*e_θ
          //   b_i = vy_ff + Ky*e_y  + Kθ*x_w*e_θ
          const double a_i = vx_ff + Kp_x_ * e_x - Kp_th_ * y_w[i] * e_th;
          const double b_i = vy_ff + Kp_y_ * e_y + Kp_th_ * x_w[i] * e_th;
          a[i] = a_i;
          b[i] = b_i;

          // --- Analytical derivatives of a_i, b_i.
          //   ȧ_i = v̇_xd − y_w*ω̇_d + Kx*ė_x  − Kθ*y_w*ė_θ
          //   ḃ_i = v̇_yd + x_w*ω̇_d + Ky*ė_y  + Kθ*x_w*ė_θ
          // Trajectory accelerations (v̇_xd, v̇_yd, ω̇_d) are not provided in
          // the Odometry reference message. We assume they are zero
          // (smooth-reference approximation). If you upgrade your
          // trajectory generator to publish accelerations, plug them in here.
          const double v_xd_dot = 0.0;
          const double v_yd_dot = 0.0;
          const double omega_d_dot = 0.0;

          const double a_dot = v_xd_dot - y_w[i] * omega_d_dot
                             + Kp_x_  * e_x_dot
                             - Kp_th_ * y_w[i] * e_th_dot;
          const double b_dot = v_yd_dot + x_w[i] * omega_d_dot
                             + Kp_y_  * e_y_dot
                             + Kp_th_ * x_w[i] * e_th_dot;

          // --- Linear wheel velocity command and its derivative.
          //   v_i = z_i * sqrt(a²+b²),   z_i ∈ {+1,-1} chosen by optimiseSpeedAngle
          //   v̇_i = z_i * (a*ȧ + b*ḃ) / sqrt(a²+b²)
          double v_lin_cmd = std::hypot(a_i, b_i);                  // magnitude
          double delta_cmd = std::atan2(b_i, a_i);

          const double norm_ab = std::max(v_lin_cmd, 1e-9);          // avoid /0
          double v_lin_dot_cmd = (a_i * a_dot + b_i * b_dot) / norm_ab;

          // optimiseSpeedAngle may flip sign of the linear velocity (and
          // shift δ by π). Apply the same sign flip to v̇.
          {
            double opt[2];
            optimiseSpeedAngle(opt, v_lin_cmd, delta_cmd);
            // detect sign flip relative to magnitude
            const double sign = (opt[0] >= 0.0) ? 1.0 : -1.0;
            v_lin_cmd     = opt[0];
            v_lin_dot_cmd = sign * v_lin_dot_cmd;
            delta_cmd     = opt[1];
          }

          // --- Convert linear -> angular for the steering-rate scaling check
          // (kept identical to the PID version so the saturation logic matches)
          double omega_cmd = v_lin_cmd / r_w[i];
          const double q = std::abs(omega_cmd / max_angular_speed);
          if (q >= maxQuotien) maxQuotien = q;

          // =========================
          // Optional LPF on v_lin_cmd (smooths reference for SMC)
          // =========================
          const double fc_omega = this->get_parameter("Fc_omega").as_double();
          if (fc_omega > 0.0) {
            const double alpha = std::exp(-2.0 * M_PI * fc_omega * dt);
            if (!omega_cmd_filt_init_) {
              omega_cmd_filt_[i] = v_lin_cmd;
            } else {
              omega_cmd_filt_[i] = alpha * omega_cmd_filt_[i] + (1.0 - alpha) * v_lin_cmd;
            }
            v_lin_cmd = omega_cmd_filt_[i];
          }

          // Optional LPF on v_dot reference (this is the value that goes
          // into the SMC equivalent control feedforward — too noisy and
          // it amplifies through B_r⁻¹).
          const double fc_vdot = this->get_parameter("Fc_vdot").as_double();
          if (fc_vdot > 0.0) {
            const double alpha = std::exp(-2.0 * M_PI * fc_vdot * dt);
            if (!v_dot_filt_init_) {
              v_dot_filt_[i] = v_lin_dot_cmd;
            } else {
              v_dot_filt_[i] = alpha * v_dot_filt_[i] + (1.0 - alpha) * v_lin_dot_cmd;
            }
            v_lin_dot_cmd = v_dot_filt_[i];
          }

          v_lin_c_i[i]     = v_lin_cmd;
          v_lin_dot_c_i[i] = v_lin_dot_cmd;
          delta_c_i[i]     = delta_cmd;

          // --- Central difference steering velocity (1-sample delay)
          const double dot_delta_raw = (delta_cmd - delta_hist_2_[i]) / (2.0 * dt);
          delta_hist_2_[i] = delta_hist_1_[i];
          delta_hist_1_[i] = delta_cmd;

          const double dot_delta_sat =
              saturate(dot_delta_raw, -max_angular_speed, max_angular_speed);

          const double fc_dot = this->get_parameter("Fc_dot").as_double();
          alpha_dot_ = std::exp(-2.0 * M_PI * fc_dot * dt);

          if (!dot_delta_filt_init_) {
            dot_delta_filt_[i] = dot_delta_sat;
          } else {
            dot_delta_filt_[i] = alpha_dot_ * dot_delta_filt_[i] + (1.0 - alpha_dot_) * dot_delta_sat;
          }
          dot_delta_c_i[i] = dot_delta_filt_[i];

          const double e_delta = (delta_cmd - delta_i_[i]);
          double steer_cmd =
              saturate(dot_delta_c_i[i] + Kp_delta_ * e_delta,
                      -max_angular_speed, max_angular_speed);

          sendingSteeringVelocity[i] = steer_cmd;
        }

        dot_delta_filt_init_ = true;
        omega_cmd_filt_init_ = true;
        v_dot_filt_init_     = true;

        // Scale velocity references uniformly if any wheel exceeds motor limit.
        // IMPORTANT: scale v_dot too, otherwise the feedforward becomes
        // inconsistent with the (now slower) reference.
        if (maxQuotien > 1.0) {
          for (int i = 0; i < 4; ++i) {
            v_lin_c_i[i]     /= maxQuotien;
            v_lin_dot_c_i[i] /= maxQuotien;
          }
        }

        // --- Publish steering velocities (unchanged)
        std_msgs::msg::Float64MultiArray steering_velocity_msg;
        for (int i = 0; i < 4; ++i) {
          steering_velocity_msg.data.push_back(sendingSteeringVelocity[i]);
        }

        // --- Publish SMC velocity reference: 8 elements
        //   [v1c..v4c, v1c_dot..v4c_dot]   units: m/s and m/s²
        // Wheel order MUST match drive_joints in smc_velocity_controller config:
        //   FL, BL, BR, FR  ↔  i=0..3 here.
        std_msgs::msg::Float64MultiArray smc_ref_msg;
        smc_ref_msg.data.reserve(8);
        for (int i = 0; i < 4; ++i) smc_ref_msg.data.push_back(v_lin_c_i[i]);
        for (int i = 0; i < 4; ++i) smc_ref_msg.data.push_back(v_lin_dot_c_i[i]);

        // --- Output / logging
        std_msgs::msg::Float64MultiArray output_msg;
        output_msg.data = {t_, x_d, y_d, theta_d, x_, y_, theta_,
                           v_lin_c_i[0], omega_i_[0] * r_w[0], tau_v_,
                           sendingSteeringVelocity[0], dot_delta_i_[0], tau_f_,
                           sendingSteeringVelocity[1], dot_delta_i_[1], tau_r_,
                           delta_c_i[0], delta_i_[0], delta_c_i[1], delta_i_[1],
                           v_lin_dot_c_i[0]};

        steering_velocity_pub_->publish(steering_velocity_msg);
        smc_velocity_ref_pub_->publish(smc_ref_msg);
        tracking_error_pub_->publish(tracking_error_msg);
        output_pub_->publish(output_msg);
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
      if (joint_index_.empty()) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
              joint_index_[msg->name[i]] = i;
          }
      }
      tau_v_ = 0.25 * (msg->effort[joint_index_["FL_wheel"]] + msg->effort[joint_index_["BL_wheel"]] + msg->effort[joint_index_["BR_wheel"]] + msg->effort[joint_index_["FR_wheel"]]);
      tau_f_ = 0.5 * (msg->effort[joint_index_["FL_steering"]] + msg->effort[joint_index_["FR_steering"]]);
      tau_r_ = 0.5 * (msg->effort[joint_index_["BL_steering"]] + msg->effort[joint_index_["BR_steering"]]);

      omega_i_[0] = msg->velocity[joint_index_["FL_wheel"]];
      omega_i_[1] = msg->velocity[joint_index_["BL_wheel"]];
      omega_i_[2] = msg->velocity[joint_index_["BR_wheel"]];
      omega_i_[3] = msg->velocity[joint_index_["FR_wheel"]];

      delta_i_[0] = msg->position[joint_index_["FL_steering"]];
      delta_i_[1] = msg->position[joint_index_["BL_steering"]];
      delta_i_[2] = msg->position[joint_index_["BR_steering"]];
      delta_i_[3] = msg->position[joint_index_["FR_steering"]];

      dot_delta_i_[0] = msg->velocity[joint_index_["FL_steering"]];
      dot_delta_i_[1] = msg->velocity[joint_index_["BL_steering"]];
      dot_delta_i_[2] = msg->velocity[joint_index_["BR_steering"]];
      dot_delta_i_[3] = msg->velocity[joint_index_["FR_steering"]];
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double roll, pitch, yaw;
        quaternionToEuler(msg->pose.pose.orientation, roll, pitch, yaw);
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        theta_ = unwrap(yaw);

        v_x_ = msg->twist.twist.linear.x;
        v_y_ = msg->twist.twist.linear.y;
        omega_ = msg->twist.twist.angular.z;
    }

    void quaternionToEuler(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw)
    {
      tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
      tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    }

    double normalizeAngle(double angle)
    {
      return fmod(angle + 2.0 * M_PI, 2.0 * M_PI);
    }

    double saturate(double x, double min_val, double max_val){
      if (x > max_val) return max_val;
      if (x < min_val) return min_val;
      return x;
    }

    double angleError(double th_d, double th)
    {
      th_d = normalizeAngle(th_d);
      th   = normalizeAngle(th);
      return fmod((th_d - th + M_PI), 2.0 * M_PI) - M_PI;
    }

    double unwrap_d(double current)
    {
        static bool initialized = false;
        static double prev = 0.0;
        static double offset = 0.0;
        const double discont = M_PI;
        const double twoPi = 2.0 * M_PI;
        if (!initialized) { prev = current; initialized = true; return current; }
        double dp = current - prev;
        if (dp > discont)       offset -= twoPi * std::ceil((dp - discont) / twoPi);
        else if (dp < -discont) offset += twoPi * std::ceil((-dp - discont) / twoPi);
        prev = current;
        return current + offset;
    }

    double unwrap(double current)
    {
        static bool initialized = false;
        static double prev = 0.0;
        static double offset = 0.0;
        const double discont = M_PI;
        const double twoPi = 2.0 * M_PI;
        if (!initialized) { prev = current; initialized = true; return current; }
        double dp = current - prev;
        if (dp > discont)       offset -= twoPi * std::ceil((dp - discont) / twoPi);
        else if (dp < -discont) offset += twoPi * std::ceil((-dp - discont) / twoPi);
        prev = current;
        return current + offset;
    }

    void optimiseSpeedAngle(double returnSpeedAngle[], double speed, double angle) const{
      if(angle > ((3 * M_PI/2))){
          returnSpeedAngle[0] = -1 * speed;
          returnSpeedAngle[1] = angle - M_PI;
      }
      else if (angle <= ((-3 * M_PI/2))){
          returnSpeedAngle[0] = -1 * speed;
          returnSpeedAngle[1] = angle + M_PI;
      }
      else{
          returnSpeedAngle[0] = speed;
          returnSpeedAngle[1] = angle;
      }
    }

    // sub
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr desired_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    // pub
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr smc_velocity_ref_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr output_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_error_pub_;

    // state
    double x_{}, y_{}, theta_{};
    double v_x_{}, v_y_{}, omega_{};
    double Kp_x_{}, Kp_y_{}, Kp_th_{}, Kp_delta_{};
    double delta_i_[4]{}, dot_delta_i_[4]{}, omega_i_[4]{};
    double last_delta_c_i_[4] = {0.0, 0.0, 0.0, 0.0};
    double last_time_traj_ = -1.0;
    double t_ = 0.0;
    double tau_v_ = 0.0;
    double tau_f_ = 0.0;
    double tau_r_ = 0.0;

    std::unordered_map<std::string, size_t> joint_index_;

    std::array<double,4> delta_hist_1_{0,0,0,0};
    std::array<double,4> delta_hist_2_{0,0,0,0};
    bool delta_hist_init_{false};

    std::array<double,4> dot_delta_filt_{0,0,0,0};
    bool dot_delta_filt_init_{false};
    double alpha_dot_{0.0};

    std::array<double,4> omega_cmd_filt_{0,0,0,0};
    bool omega_cmd_filt_init_{false};

    std::array<double,4> v_dot_filt_{0,0,0,0};
    bool v_dot_filt_init_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelBasedControllerVelocity>());
    rclcpp::shutdown();
    return 0;
}