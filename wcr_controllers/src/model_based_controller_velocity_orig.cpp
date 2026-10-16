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
#include <control_msgs/msg/multi_dof_command.hpp>

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
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);

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

        RCLCPP_INFO(this->get_logger(), "ModelBasedControllerVelocity node started");
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
        
        //velocities are recived transformed in robot base_link frame
        double v_x_d = msg->twist.twist.linear.x;
        double v_y_d = msg->twist.twist.linear.y;
        double omega_d = msg->twist.twist.angular.z;

        //Error transformed in robot frame
        double e_x = (x_d - x_) * cos(theta_) + (y_d - y_) * sin(theta_);
        double e_y = -(x_d - x_) * sin(theta_) + (y_d - y_) * cos(theta_);
        double e_th = theta_d - theta_;
        double x_w[4] = {this->get_parameter("x_w_1").as_double(), this->get_parameter("x_w_2").as_double(), this->get_parameter("x_w_3").as_double(), this->get_parameter("x_w_4").as_double()};
        double y_w[4] = {this->get_parameter("y_w_1").as_double(), this->get_parameter("y_w_2").as_double(), this->get_parameter("y_w_3").as_double(), this->get_parameter("y_w_4").as_double()};
        double r_w[4] = {this->get_parameter("r_w_1").as_double(), this->get_parameter("r_w_2").as_double(), this->get_parameter("r_w_3").as_double(), this->get_parameter("r_w_4").as_double()};
        Kp_x_ = this->get_parameter("Kp_x").as_double();
        Kp_y_ = this->get_parameter("Kp_y").as_double();
        Kp_th_ = this->get_parameter("Kp_th").as_double();
        Kp_delta_ = this->get_parameter("Kp_delta").as_double();

        double v_x_i[4], v_y_i[4], v_d_i[4], delta_d_i[4];
        double a[4], b[4], omega_c_i[4], delta_c_i[4], dot_delta_c_i[4];
        double max_angular_speed = (210*0.229) * ((2.0*M_PI)/60.0);
        double maxQuotien = 0.0;
        double sendingSteeringVelocity[4];
        //RCLCPP_INFO(this->get_logger(), "Error X: %f", e_x);
        //RCLCPP_INFO(this->get_logger(), "Error Y: %f", e_y);
        //RCLCPP_INFO(this->get_logger(), "Correction X: %f", e_x*Kp_x_);
        //RCLCPP_INFO(this->get_logger(), "Correciton Y: %f", e_y*Kp_x_);

        // ---- One-time init of histories/filters (do this before the for-loop)
                if (!delta_hist_init_) {
          for (int i = 0; i < 4; ++i) {
            // initialize with current measured steering (safe default)
            delta_hist_1_[i] = delta_i_[i];
            delta_hist_2_[i] = delta_i_[i];
          }
          delta_hist_init_ = true;
        }

        // ---- Main loop
        for (int i = 0; i < 4; ++i) {

          // --- Cartesian -> wheel-space velocities
          const double vx = v_x_d - y_w[i] * omega_d;
          const double vy = v_y_d + x_w[i] * omega_d;

          const double v_des     = std::hypot(vx, vy);
          const double delta_des = std::atan2(vy, vx);

          v_x_i[i]     = vx;
          v_y_i[i]     = vy;
          v_d_i[i]     = v_des;
          delta_d_i[i] = delta_des;

          // --- Controller in wheel space
          // v_des*cos(delta_des)=vx and v_des*sin(delta_des)=vy, so avoid trig recomputation
          const double a_i = vx + Kp_x_ * e_x - Kp_th_ * y_w[i] * e_th;
          const double b_i = vy + Kp_y_ * e_y + Kp_th_ * x_w[i] * e_th;

          a[i] = a_i;
          b[i] = b_i;

          // --- Commands (pre-optimization)
          double omega_cmd = std::hypot(a_i, b_i) / r_w[i];
          double delta_cmd = std::atan2(b_i, a_i);

          // --- Check if velocity exceeds maximum motor velocity (same logic as yours)
          const double q = std::abs(omega_cmd / max_angular_speed);
          if (q >= maxQuotien) maxQuotien = q;

          // --- Optimise speed/angle (keep your function)
          {
            double opt[2];
            optimiseSpeedAngle(opt, omega_cmd, delta_cmd);
            omega_cmd = opt[0];
            delta_cmd = opt[1];
          }

          // =========================
          // ADD: Low-pass filter omega_cmd (driving velocity)
          // =========================
          const double fc_omega = this->get_parameter("Fc_omega").as_double();   // e.g., start 5–15 Hz @ 200 Hz
          alpha_omega_ = std::exp(-2.0 * M_PI * fc_omega * dt);

          if (!omega_cmd_filt_init_) {
            omega_cmd_filt_[i] = omega_cmd;
          } else {
            omega_cmd_filt_[i] = alpha_omega_ * omega_cmd_filt_[i] + (1.0 - alpha_omega_) * omega_cmd;
          }

          omega_cmd = omega_cmd_filt_[i];
          // =========================

          omega_c_i[i] = omega_cmd;
          delta_c_i[i] = delta_cmd;

          // --- Central difference steering velocity (1-sample delay)
          // dot_delta[k-1] = (delta[k] - delta[k-2]) / (2 dt)
          const double dot_delta_raw = (delta_cmd - delta_hist_2_[i]) / (2.0 * dt);

          // shift history AFTER computing derivative
          delta_hist_2_[i] = delta_hist_1_[i];
          delta_hist_1_[i] = delta_cmd;

          // saturate raw derivative
          const double dot_delta_sat =
              saturate(dot_delta_raw, -max_angular_speed, max_angular_speed);

          // --- Low-pass filter derivative (1st order IIR)
          const double fc_dot = this->get_parameter("Fc_dot").as_double(); // start 5–10 Hz at 200 Hz
          alpha_dot_ = std::exp(-2.0 * M_PI * fc_dot * dt);

          if (!dot_delta_filt_init_) {
            dot_delta_filt_[i] = dot_delta_sat;
          } else {
            dot_delta_filt_[i] = alpha_dot_ * dot_delta_filt_[i] + (1.0 - alpha_dot_) * dot_delta_sat;
          }

          dot_delta_c_i[i] = dot_delta_filt_[i];

          // --- Steering velocity command
          const double e_delta = (delta_cmd - delta_i_[i]);
          double steer_cmd =
              saturate(dot_delta_c_i[i] + Kp_delta_ * e_delta,
                      -max_angular_speed, max_angular_speed);

          sendingSteeringVelocity[i] = steer_cmd;
        }

        dot_delta_filt_init_ = true;

        // ADD: mark omega filter initialized
        omega_cmd_filt_init_ = true;



        if (maxQuotien > 1.0){
          for(int i=0; i<4; i++){
            omega_c_i[i] /= maxQuotien;
          }
        }
        std_msgs::msg::Float64MultiArray velocity_msg;
        for (int i=0;i<4;i++){
          velocity_msg.data.push_back(omega_c_i[i]);
        }
        for (int i=0;i<4;i++){
          velocity_msg.data.push_back(sendingSteeringVelocity[i]);
        }

        std_msgs::msg::Float64MultiArray output_msg;
        //output_msg.data = {t_, omega_c_i[0] * r_w[0], sendingSteeringVelocity[0], sendingSteeringVelocity[1],
        //   omega_i_[0] * r_w[0], dot_delta_i_[0], dot_delta_i_[1], tau_v_, tau_f_, tau_r_, x_d, y_d, theta_d, x_, y_, theta_};
        output_msg.data = {t_, x_d, y_d, theta_d, x_, y_, theta_,  omega_c_i[0] * r_w[0], omega_i_[0] * r_w[0], tau_v_, sendingSteeringVelocity[0], dot_delta_i_[0], tau_f_, sendingSteeringVelocity[1], dot_delta_i_[1], tau_r_, delta_c_i[0], delta_i_[0], delta_c_i[1], delta_i_[1]};
        velocity_pub_->publish(velocity_msg);
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
    // normalize both
      th_d = normalizeAngle(th_d);
      th   = normalizeAngle(th);

    // compute shortest difference in [-pi, pi]
      return fmod((th_d - th + M_PI), 2.0 * M_PI) - M_PI;
    } 

    double unwrap_d(double current)
    {
        static bool initialized = false;
        static double prev = 0.0;
        static double offset = 0.0;

        const double discont = M_PI;
        const double twoPi = 2.0 * M_PI;

        if (!initialized) {
            prev = current;
            initialized = true;
            return current;
        }

        double dp = current - prev;

        if (dp > discont) {
            offset -= twoPi * std::ceil((dp - discont) / twoPi);
        }
        else if (dp < -discont) {
            offset += twoPi * std::ceil((-dp - discont) / twoPi);
        }

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

        if (!initialized) {
            prev = current;
            initialized = true;
            return current;
        }

        double dp = current - prev;

        if (dp > discont) {
            offset -= twoPi * std::ceil((dp - discont) / twoPi);
        }
        else if (dp < -discont) {
            offset += twoPi * std::ceil((-dp - discont) / twoPi);
        }

        prev = current;
        return current + offset;
    }

    double front_steering_velocity_filtered(double delta_now, double dt)
    {
        static bool initialized = false;
        static double delta_prev = 0.0;
        static double dot_delta_f = 0.0;

        // ---- tuning parameters ----
        const double fc = 5.0;          // cutoff frequency [Hz] (3–8 Hz recommended)
        const double alpha = std::exp(-2.0 * M_PI * fc * dt);

        if (!initialized) {
            delta_prev = delta_now;
            initialized = true;
            return 0.0;
        }

        // raw derivative
        double dot_delta_raw = (delta_now - delta_prev) / dt;
        delta_prev = delta_now;

        // first-order low-pass filter
        dot_delta_f = alpha * dot_delta_f + (1.0 - alpha) * dot_delta_raw;

        return dot_delta_f;  // rad/s
    }

    double rear_steering_velocity_filtered(double delta_now, double dt)
    {
        static bool initialized = false;
        static double delta_prev = 0.0;
        static double dot_delta_f = 0.0;

        // ---- tuning parameters ----
        const double fc = 5.0;          // cutoff frequency [Hz] (3–8 Hz recommended)
        const double alpha = std::exp(-2.0 * M_PI * fc * dt);

        if (!initialized) {
            delta_prev = delta_now;
            initialized = true;
            return 0.0;
        }

        // raw derivative
        double dot_delta_raw = (delta_now - delta_prev) / dt;
        delta_prev = delta_now;

        // first-order low-pass filter
        dot_delta_f = alpha * dot_delta_f + (1.0 - alpha) * dot_delta_raw;

        return dot_delta_f;  // rad/s
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
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr output_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_error_pub_;

    // state variables
    double x_, y_, theta_;
    double v_x_, v_y_, omega_;
    double Kp_x_, Kp_y_, Kp_th_, Kp_delta_;
    double delta_i_[4], dot_delta_i_[4], omega_i_[4];
    double last_delta_c_i_[4] = {0.0, 0.0, 0.0, 0.0};
    double last_time_traj_ = -1.0;
    double t_ = 0.0;
    double tau_v_ = 0.0;
    double tau_f_ = 0.0;
    double tau_r_ = 0.0;

    double theta_prev_ = 0.0;
    double offset_ = 0.0;

    double theta_d_prev_ = 0.0;
    double offset_d_ = 0.0;

    std::unordered_map<std::string, size_t> joint_index_;

    std::array<double,4> delta_hist_1_{0,0,0,0};  // delta[k-1]
    std::array<double,4> delta_hist_2_{0,0,0,0};  // delta[k-2]
    bool delta_hist_init_{false};

    std::array<double,4> dot_delta_filt_{0,0,0,0};
    bool dot_delta_filt_init_{false};
    double alpha_dot_{0.0};   // LPF coefficient
    std::array<double,4> omega_cmd_filt_{0,0,0,0};
    bool omega_cmd_filt_init_{false};
    double alpha_omega_{0.0};
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelBasedControllerVelocity>());
    auto node = std::make_shared<ModelBasedControllerVelocity>();
    return 0;
}