#include <cmath>
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

using namespace std::chrono_literals;

class ReducedModelBasedControllerVelocity : public rclcpp::Node
{
public:
    ReducedModelBasedControllerVelocity()
    : Node("trajectory_follower")
    {
        // sub
        desired_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wcr/desired_trajectory", 10,
            std::bind(&ReducedModelBasedControllerVelocity::desired_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wcr/odom", 10,
            std::bind(&ReducedModelBasedControllerVelocity::odom_callback, this, std::placeholders::_1));
        
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&ReducedModelBasedControllerVelocity::joint_states_callback, this, std::placeholders::_1));

        // pub
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);

        tracking_error_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/wcr/tracking_error", 10);

        steering_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);

        //timeout timer
        trajectory_timeout_timer_ = this->create_wall_timer(100ms, std::bind(&ReducedModelBasedControllerVelocity::checkTrajectoryTimeout, this));
        //trajectory_timeout_timer_ = this->create_wall_timer(100ms, std::bind(&ReducedModelBasedControllerVelocity::checkTrajectoryTimeout, this));

        // parameters
        this->declare_parameter("x_w_1", 0.1125);
        this->declare_parameter("y_w_1", 0.1125);
        this->declare_parameter("r_w_1", 0.0254);

        this->declare_parameter("x_w_2", -0.1125);
        this->declare_parameter("y_w_2", 0.1125);
        this->declare_parameter("r_w_2", 0.0254);

        this->declare_parameter("Kp_x", 5.0);
        this->declare_parameter("Kp_y", 5.0);
        this->declare_parameter("Kp_th", 3.0);
        this->declare_parameter("Kp_delta", 2.0);

        RCLCPP_INFO(this->get_logger(), "ReducedModelBasedControllerVelocity node started");
    }

private:
    void checkTrajectoryTimeout()
      {
        if (recived_first_trajectory_data_ == false)
          return;
        // Compare trajectory to previous values
        bool changed = (x_d_ != last_x_d_) || (y_d_ != last_y_d_) || (theta_d_ != last_theta_d_);

        if (changed)
        {
          last_x_d_ = x_d_;
          last_y_d_ = y_d_;
          last_theta_d_ = theta_d_;
          last_trajectory_update_time_ = this->now();
        }
        else
        {
          auto elapsed = (this->now() - last_trajectory_update_time_).seconds();
          if (elapsed > 0.2)
          {
            //RCLCPP_WARN(this->get_logger(), "Trajectory has not changed for %.2f seconds!", elapsed);
            std_msgs::msg::Float64MultiArray velocity_msg;
            velocity_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            velocity_pub_->publish(velocity_msg);

          }
        }
      }
    void desired_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(last_time_traj_ < 0){
          rclcpp::Time temp_time = msg->header.stamp;
          last_time_traj_ = temp_time.seconds();
          return;
        }
        recived_first_trajectory_data_ = true;
        rclcpp::Time temp_current_time = msg->header.stamp;
        double dt = temp_current_time.seconds() - last_time_traj_;
        last_time_traj_ = temp_current_time.seconds();

        double roll_d, pitch_d, yaw_d;
        quaternionToEuler(msg->pose.pose.orientation, roll_d, pitch_d, yaw_d);
        x_d_ = msg->pose.pose.position.x;
        y_d_ = msg->pose.pose.position.y;
        theta_d_ = unwrapAngle(yaw_d, theta_prev_, theta_offset_);

        geometry_msgs::msg::PoseStamped tracking_error_msg;
        tracking_error_msg.header.stamp = temp_current_time;
        tracking_error_msg.pose.position.x = abs(x_d_ - x_);
        tracking_error_msg.pose.position.y = abs(y_d_ - y_);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_d_ - theta_);
        q.normalize();
        tracking_error_msg.pose.orientation = tf2::toMsg(q);
        
        //velocities are recived transformed in robot base_link frame
        double v_x_d = msg->twist.twist.linear.x;
        double v_y_d = msg->twist.twist.linear.y;
        double omega_d = msg->twist.twist.angular.z;

        //Error transformed in robot frame
        double e_x = (x_d_ - x_) * cos(theta_) + (y_d_ - y_) * sin(theta_);
        double e_y = -(x_d_ - x_) * sin(theta_) + (y_d_ - y_) * cos(theta_);
        double e_th = angleError(theta_d_, theta_);
        double x_w[2] = {this->get_parameter("x_w_1").as_double(), this->get_parameter("x_w_2").as_double()};
        double y_w[2] = {this->get_parameter("y_w_1").as_double(), this->get_parameter("y_w_2").as_double()};
        double r_w[2] = {this->get_parameter("r_w_1").as_double(), this->get_parameter("r_w_2").as_double()};
        Kp_x_ = this->get_parameter("Kp_x").as_double();
        Kp_y_ = this->get_parameter("Kp_y").as_double();
        Kp_th_ = this->get_parameter("Kp_th").as_double();
        Kp_delta_ = this->get_parameter("Kp_delta").as_double();

        double v_x_i[4], v_y_i[4], v_d_i[4], delta_d_i[4];
        double a[4], b[4], omega_c_i[4], delta_c_i[4], dot_delta_c_i[4];
        double max_angular_speed = (210*0.229) * ((2.0*M_PI)/60.0);
        double maxQuotien = 0.0;
        double sendingSteeringVelocity[4];

        for(int i=0;i<2;i++){
          //transform trajectory from cartesian space to robot joint space
          v_x_i[i] = v_x_d - y_w[i] * omega_d;
          v_y_i[i] = v_y_d + x_w[i] * omega_d;
          v_d_i[i] = sqrt(v_x_i[i] * v_x_i[i] + v_y_i[i] * v_y_i[i]);
          delta_d_i[i] = atan2(v_y_i[i], v_x_i[i]);
          
          a[i] = v_d_i[i] * cos(delta_d_i[i]) + Kp_x_ * e_x - Kp_th_ * y_w[i] * e_th;
          b[i] = v_d_i[i] * sin(delta_d_i[i]) + Kp_y_ * e_y + Kp_th_ * x_w[i] * e_th;

          //calculating steering angle and driving velocity
          omega_c_i[i] = (sqrt(a[i] * a[i] + b[i] * b[i])) / r_w[i];
          delta_c_i[i] = atan2(b[i], a[i]);

          //checking if velocity is higher then maximum motor velocity
          double quotien=std::abs(omega_c_i[i]/max_angular_speed);
          if (quotien >= maxQuotien)
            maxQuotien = quotien;

          double returnValueTemp[2];
          optimiseSpeedAngle(returnValueTemp, omega_c_i[i], delta_c_i[i]);
          omega_c_i[i] = returnValueTemp[0];
          delta_c_i[i] = returnValueTemp[1];

          dot_delta_c_i[i] = (delta_c_i[i] - last_delta_c_i_[i])/dt;
          last_delta_c_i_[i] = delta_c_i[i];

          sendingSteeringVelocity[i] = saturate(dot_delta_c_i[i] + Kp_delta_*(delta_c_i[i] - delta_i_[i]), -max_angular_speed, max_angular_speed);

        }

        if (maxQuotien > 1.0){
          for(int i=0; i<2; i++){
            omega_c_i[i] /= maxQuotien;
          }
        }
        std_msgs::msg::Float64MultiArray velocity_msg;
        velocity_msg.data.push_back(omega_c_i[0]);
        velocity_msg.data.push_back(omega_c_i[1]);
        velocity_msg.data.push_back(omega_c_i[1]);
        velocity_msg.data.push_back(omega_c_i[0]);
        velocity_msg.data.push_back(sendingSteeringVelocity[0]);
        velocity_msg.data.push_back(sendingSteeringVelocity[1]);
        velocity_msg.data.push_back(sendingSteeringVelocity[1]);
        velocity_msg.data.push_back(sendingSteeringVelocity[0]);

        std_msgs::msg::Float64MultiArray delta_c_msg;
        delta_c_msg.data.push_back(delta_c_i[0]);
        delta_c_msg.data.push_back(delta_c_i[1]);
        delta_c_msg.data.push_back(delta_c_i[1]);
        delta_c_msg.data.push_back(delta_c_i[0]);

        velocity_pub_->publish(velocity_msg);
        tracking_error_pub_->publish(tracking_error_msg);
        steering_pub_->publish(delta_c_msg);
        
    }

    double saturate(double value, double min_val, double max_val) {
      if (value > max_val) return max_val;
      if (value < min_val) return min_val;
    return value;
    }

    double unwrapAngle(double theta, double &theta_prev, double &offset) {
      // Detect jump across -pi/pi
      double diff = theta - theta_prev;
      if (diff > M_PI) {
          offset -= 2.0 * M_PI;
      } else if (diff < -M_PI) {
          offset += 2.0 * M_PI;
      }

      theta_prev = theta;
      return theta + offset;
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
      if (joint_index_.empty()) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
              joint_index_[msg->name[i]] = i;
          }
      }
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
        theta_ = yaw;

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

    double angleError(double th_d, double th)
    {
    // normalize both
      th_d = normalizeAngle(th_d);
      th   = normalizeAngle(th);

    // compute shortest difference in [-pi, pi]
      return fmod((th_d - th + M_PI), 2.0 * M_PI) - M_PI;
    } 

    void optimiseSpeedAngle(double returnSpeedAngle[], double speed, double angle) const{ 
      if(angle > (M_PI/2)){
          returnSpeedAngle[0] = -1 * speed;
          returnSpeedAngle[1] = angle - M_PI;
      }
      else if (angle <= (-M_PI/2)){
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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;

    rclcpp::TimerBase::SharedPtr trajectory_timeout_timer_;
    


    // state variables
    double x_, y_, theta_;
    double v_x_, v_y_, omega_;
    double Kp_x_, Kp_y_, Kp_th_, Kp_delta_;
    double delta_i_[4], dot_delta_i_[4];
    double last_delta_c_i_[4] = {0.0, 0.0, 0.0, 0.0};
    double last_time_traj_ = -1.0;
    double theta_prev_ = 0.0;
    double theta_offset_ = 0.0;

    double x_d_, y_d_, theta_d_;
    double last_x_d_, last_y_d_, last_theta_d_;
    rclcpp::Time last_trajectory_update_time_;

    bool recived_first_trajectory_data_ = false;

    std::unordered_map<std::string, size_t> joint_index_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReducedModelBasedControllerVelocity>());
    auto node = std::make_shared<ReducedModelBasedControllerVelocity>();
    return 0;
}
