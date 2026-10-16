#include <cmath>
#include <array>
#include <algorithm>
#include <unordered_map>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class ModelBasedControllerVelocity : public rclcpp::Node
{
public:
    ModelBasedControllerVelocity()
    : Node("trajectory_follower")
    {
        timer_ = this->create_wall_timer(5ms, std::bind(&ModelBasedControllerVelocity::control_loop, this));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
            std::bind(&ModelBasedControllerVelocity::joyCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wcr/odom", 10,
            std::bind(&ModelBasedControllerVelocity::odom_callback, this, std::placeholders::_1));

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&ModelBasedControllerVelocity::joint_states_callback, this, std::placeholders::_1));

        // pub
        steering_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/steering_velocity_controller/commands", 10);

        // SMC velocity reference: [v1c..v4c, v1c_dot..v4c_dot]  (m/s, m/s^2)
        // Wheel order must match drive_joints of smc_velocity_controller: FL, BL, BR, FR
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
        this->declare_parameter("Fc_omega", 5.0);   // LPF on linear wheel velocity ref (Hz), 0 = off
        this->declare_parameter("Fc_vdot", 20.0);   // LPF on v_dot ref (Hz), 0 = off

        RCLCPP_INFO(this->get_logger(), "Figure-eight follower (SMC) node started");
        RCLCPP_INFO(this->get_logger(), "Press X to start");
    }

private:
    enum class Phase { RUN, PAUSE, RUN_NO_FB, DONE };

    // Steering hold angle during pause / end. Slightly above -pi/2 on purpose,
    // so optimiseSpeedAngle does NOT flip it to +pi/2.
    static constexpr double kDeltaHold = -1.57079632679;

    void publishZeros()
    {
        std_msgs::msg::Float64MultiArray steer;
        steer.data = {0.0, 0.0, 0.0, 0.0};
        std_msgs::msg::Float64MultiArray smc;
        smc.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        steering_velocity_pub_->publish(steer);
        smc_velocity_ref_pub_->publish(smc);
    }

    void control_loop()
    {
        const rclcpp::Time now_t = this->now();

        // ---- E-stop / not ready: hold zero references
        if (estop_pressed_ || !odom_received_ || !joints_received_) {
          if (!estop_pressed_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Waiting for odom (%d) and joint_states (%d)",
                                 odom_received_, joints_received_);
          }
          publishZeros();
          last_time_traj_ = now_t.seconds();
          return;
        }

        if (last_time_traj_ < 0) {
          last_time_traj_ = now_t.seconds();
          return;
        }
        const double dt = now_t.seconds() - last_time_traj_;
        last_time_traj_ = now_t.seconds();
        if (dt <= 0.0) return;
        t_ += dt;

        // ---- Timing / phases
        const double t_pause = 10.0 * M_PI;             // half of the eight (v*t = pi)
        const double T_pause = 1.0;
        const double T_fb_off = 0.2;                    // feedback off after pause
        const double t_end   = 2.0 * t_pause + T_pause; // full eight done

        Phase phase;
        if (t_ < t_pause)                               phase = Phase::RUN;
        else if (t_ < t_pause + T_pause)                phase = Phase::PAUSE;
        else if (t_ <= t_pause + T_pause + T_fb_off)    phase = Phase::RUN_NO_FB;
        else if (t_ < t_end)                            phase = Phase::RUN;
        else                                            phase = Phase::DONE;

        // trajectory time (frozen during pause, shifted after it, clamped at end)
        double tau;
        if (t_ < t_pause)                  tau = t_;
        else if (t_ < t_pause + T_pause)   tau = t_pause;
        else                               tau = std::min(t_ - T_pause, 2.0 * t_pause);

        const bool moving   = (phase == Phase::RUN || phase == Phase::RUN_NO_FB);
        const bool feedback = (phase == Phase::RUN);

        // ---- Figure-eight evaluated at tau (world / wall frame)
        const double c = 0.5;
        const double d = 0.75;
        const double v = 0.1;
        const double phi = -M_PI / 2.0;     // start at (0, 0)
        const double s   = v * tau + phi;

        const double x_d = d * std::sin(s) + d;
        const double y_d = -c * std::sin(2.0 * s);
        const double theta_d = 0.0;

        double v_x_d_w = 0.0, v_y_d_w = 0.0, a_x_d_w = 0.0, a_y_d_w = 0.0;
        if (moving) {
          v_x_d_w =  d * v * std::cos(s);
          v_y_d_w = -2.0 * c * v * std::cos(2.0 * s);
          a_x_d_w = -d * v * v * std::sin(s);
          a_y_d_w =  4.0 * c * v * v * std::sin(2.0 * s);
        }
        const double omega_d = 0.0;
        const double omega_d_dot = 0.0;

        // Feedforward rotated into robot frame (identity when theta ~ 0,
        // consistent with the error rotation below).
        const double ct = std::cos(theta_), st = std::sin(theta_);
        const double v_x_d    =  ct * v_x_d_w + st * v_y_d_w;
        const double v_y_d    = -st * v_x_d_w + ct * v_y_d_w;
        const double v_xd_dot =  ct * a_x_d_w + st * a_y_d_w;
        const double v_yd_dot = -st * a_x_d_w + ct * a_y_d_w;

        geometry_msgs::msg::PoseStamped tracking_error_msg;
        tracking_error_msg.header.stamp = now_t;
        tracking_error_msg.pose.position.x = x_d - x_;
        tracking_error_msg.pose.position.y = y_d - y_;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_d - theta_);
        q.normalize();
        tracking_error_msg.pose.orientation = tf2::toMsg(q);

        // ---- Errors in robot frame
        const double e_x  =  (x_d - x_) * ct + (y_d - y_) * st;
        const double e_y  = -(x_d - x_) * st + (y_d - y_) * ct;
        const double e_th = theta_d - theta_;

        // Error derivatives (odom twist assumed in robot frame)
        const double e_x_dot  = v_x_d - v_x_;
        const double e_y_dot  = v_y_d - v_y_;
        const double e_th_dot = omega_d - omega_;

        double x_w[4] = {this->get_parameter("x_w_1").as_double(), this->get_parameter("x_w_2").as_double(),
                         this->get_parameter("x_w_3").as_double(), this->get_parameter("x_w_4").as_double()};
        double y_w[4] = {this->get_parameter("y_w_1").as_double(), this->get_parameter("y_w_2").as_double(),
                         this->get_parameter("y_w_3").as_double(), this->get_parameter("y_w_4").as_double()};
        double r_w[4] = {this->get_parameter("r_w_1").as_double(), this->get_parameter("r_w_2").as_double(),
                         this->get_parameter("r_w_3").as_double(), this->get_parameter("r_w_4").as_double()};
        Kp_x_     = this->get_parameter("Kp_x").as_double();
        Kp_y_     = this->get_parameter("Kp_y").as_double();
        Kp_th_    = this->get_parameter("Kp_th").as_double();
        Kp_delta_ = this->get_parameter("Kp_delta").as_double();
        const double fc_omega = this->get_parameter("Fc_omega").as_double();
        const double fc_vdot  = this->get_parameter("Fc_vdot").as_double();
        const double fc_dot   = this->get_parameter("Fc_dot").as_double();

        double v_lin_c_i[4]{}, v_lin_dot_c_i[4]{}, delta_c_i[4]{}, dot_delta_c_i[4]{};
        double sendingSteeringVelocity[4]{};
        const double max_angular_speed = (210 * 0.229) * ((2.0 * M_PI) / 60.0);
        double maxQuotien = 0.0;

        if (!delta_hist_init_) {
          for (int i = 0; i < 4; ++i) {
            delta_hist_1_[i] = delta_i_[i];
            delta_hist_2_[i] = delta_i_[i];
          }
          delta_hist_init_ = true;
        }

        const double fb = feedback ? 1.0 : 0.0;

        for (int i = 0; i < 4; ++i) {

          // --- Feedforward in wheel space
          const double vx     = v_x_d - y_w[i] * omega_d;
          const double vy     = v_y_d + x_w[i] * omega_d;
          const double vx_dot = v_xd_dot - y_w[i] * omega_d_dot;
          const double vy_dot = v_yd_dot + x_w[i] * omega_d_dot;

          // --- Controller in wheel space
          const double a_i = vx + fb * (Kp_x_ * e_x - Kp_th_ * y_w[i] * e_th);
          const double b_i = vy + fb * (Kp_y_ * e_y + Kp_th_ * x_w[i] * e_th);
          const double a_dot = vx_dot + fb * (Kp_x_ * e_x_dot - Kp_th_ * y_w[i] * e_th_dot);
          const double b_dot = vy_dot + fb * (Kp_y_ * e_y_dot + Kp_th_ * x_w[i] * e_th_dot);

          double v_lin_cmd, v_lin_dot_cmd, delta_cmd;

          if (phase == Phase::PAUSE || phase == Phase::DONE) {
            v_lin_cmd     = 0.0;
            v_lin_dot_cmd = 0.0;
            delta_cmd     = kDeltaHold;
          } else {
            v_lin_cmd = std::hypot(a_i, b_i);
            delta_cmd = std::atan2(b_i, a_i);
            const double norm_ab = std::max(v_lin_cmd, 1e-9);
            v_lin_dot_cmd = (a_i * a_dot + b_i * b_dot) / norm_ab;
          }

          // --- Motor limit check (angular wheel speed)
          const double qq = std::abs((v_lin_cmd / r_w[i]) / max_angular_speed);
          if (qq >= maxQuotien) maxQuotien = qq;

          // --- Optimise speed/angle; apply same sign flip to v_dot
          {
            double opt[2];
            optimiseSpeedAngle(opt, v_lin_cmd, delta_cmd);
            const bool flipped = (opt[1] != delta_cmd);
            v_lin_cmd = opt[0];
            delta_cmd = opt[1];
            if (flipped) v_lin_dot_cmd = -v_lin_dot_cmd;
          }

          // --- LPF on linear velocity reference
          if (fc_omega > 0.0) {
            const double alpha = std::exp(-2.0 * M_PI * fc_omega * dt);
            if (!omega_cmd_filt_init_) omega_cmd_filt_[i] = v_lin_cmd;
            else omega_cmd_filt_[i] = alpha * omega_cmd_filt_[i] + (1.0 - alpha) * v_lin_cmd;
            v_lin_cmd = omega_cmd_filt_[i];
          }

          // --- LPF on v_dot reference (SMC equivalent-control feedforward)
          if (fc_vdot > 0.0) {
            const double alpha = std::exp(-2.0 * M_PI * fc_vdot * dt);
            if (!v_dot_filt_init_) v_dot_filt_[i] = v_lin_dot_cmd;
            else v_dot_filt_[i] = alpha * v_dot_filt_[i] + (1.0 - alpha) * v_lin_dot_cmd;
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

          alpha_dot_ = std::exp(-2.0 * M_PI * fc_dot * dt);
          if (!dot_delta_filt_init_) dot_delta_filt_[i] = dot_delta_sat;
          else dot_delta_filt_[i] = alpha_dot_ * dot_delta_filt_[i] + (1.0 - alpha_dot_) * dot_delta_sat;
          dot_delta_c_i[i] = dot_delta_filt_[i];

          // --- Steering velocity command
          const double e_delta = delta_cmd - delta_i_[i];
          sendingSteeringVelocity[i] =
              saturate(dot_delta_c_i[i] + Kp_delta_ * e_delta,
                       -max_angular_speed, max_angular_speed);
        }

        dot_delta_filt_init_ = true;
        omega_cmd_filt_init_ = true;
        v_dot_filt_init_     = true;

        // --- Uniform scaling if any wheel exceeds motor limit (scale v_dot too)
        if (maxQuotien > 1.0) {
          for (int i = 0; i < 4; ++i) {
            v_lin_c_i[i]     /= maxQuotien;
            v_lin_dot_c_i[i] /= maxQuotien;
          }
        }

        // --- Publish steering velocities
        std_msgs::msg::Float64MultiArray steering_velocity_msg;
        for (int i = 0; i < 4; ++i) steering_velocity_msg.data.push_back(sendingSteeringVelocity[i]);

        // --- Publish SMC reference [v1c..v4c, v1c_dot..v4c_dot], order FL, BL, BR, FR
        std_msgs::msg::Float64MultiArray smc_ref_msg;
        smc_ref_msg.data.reserve(8);
        for (int i = 0; i < 4; ++i) smc_ref_msg.data.push_back(v_lin_c_i[i]);
        for (int i = 0; i < 4; ++i) smc_ref_msg.data.push_back(v_lin_dot_c_i[i]);

        // --- Output / logging (same layout as the desired-trajectory SMC node)
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

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      if (joint_index_.empty()) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
              joint_index_[msg->name[i]] = i;
          }
      }
      tau_v_ = 0.25 * (msg->effort[joint_index_["FL_wheel"]] + msg->effort[joint_index_["BL_wheel"]] +
                       msg->effort[joint_index_["BR_wheel"]] + msg->effort[joint_index_["FR_wheel"]]);
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

      joints_received_ = true;
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons.empty()) return;
        const bool button0 = (msg->buttons[0] == 1);

        if (button0 && !last_button0_) {
            estop_pressed_ = !estop_pressed_;   // toggle
            RCLCPP_WARN(this->get_logger(), "E-STOP %s", estop_pressed_ ? "ENGAGED" : "RELEASED");
        }
        last_button0_ = button0;
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

        odom_received_ = true;
    }

    void quaternionToEuler(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw)
    {
      tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
      tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    }

    double saturate(double x, double min_val, double max_val)
    {
      if (x > max_val) return max_val;
      if (x < min_val) return min_val;
      return x;
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

    // Wall version: keeps steering within (-pi/2, pi/2], flips wheel speed otherwise
    void optimiseSpeedAngle(double returnSpeedAngle[], double speed, double angle) const
    {
      if (angle > (M_PI / 2)) {
          returnSpeedAngle[0] = -speed;
          returnSpeedAngle[1] = angle - M_PI;
      } else if (angle <= (-M_PI / 2)) {
          returnSpeedAngle[0] = -speed;
          returnSpeedAngle[1] = angle + M_PI;
      } else {
          returnSpeedAngle[0] = speed;
          returnSpeedAngle[1] = angle;
      }
    }

    // sub
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // pub
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr smc_velocity_ref_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr output_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_error_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // state
    double x_{}, y_{}, theta_{};
    double v_x_{}, v_y_{}, omega_{};
    double Kp_x_{}, Kp_y_{}, Kp_th_{}, Kp_delta_{};
    double delta_i_[4]{}, dot_delta_i_[4]{}, omega_i_[4]{};
    double last_time_traj_ = -1.0;
    double t_ = 0.0;
    double tau_v_ = 0.0, tau_f_ = 0.0, tau_r_ = 0.0;

    bool odom_received_{false};
    bool joints_received_{false};

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

    bool estop_pressed_ = true;
    bool last_button0_  = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelBasedControllerVelocity>());
    rclcpp::shutdown();
    return 0;
}