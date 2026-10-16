#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class ModelBasedController : public rclcpp::Node
{
public:
    ModelBasedController()
    : Node("trajectory_follower")
    {
        // sub
        desired_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wcr/desired_trajectory", 10,
            std::bind(&ModelBasedController::desired_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "wcr/odom", 10,
            std::bind(&ModelBasedController::odom_callback, this, std::placeholders::_1));

        // pub
        driving_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/driving_velocity_controller/commands", 10);

        steering_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/steering_position_controller/commands", 10);

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
        this->declare_parameter("Kp_th", 2.0);

        RCLCPP_INFO(this->get_logger(), "ModelBasedController node started");
    }
    ~ModelBasedController()
    {
      RCLCPP_INFO(this->get_logger(), "Shutting down, sending zero commands...");

      std_msgs::msg::Float64MultiArray zero_drive;
      std_msgs::msg::Float64MultiArray zero_steer;

      // Four wheels → push 4 zeros
      zero_drive.data = {0.0, 0.0, 0.0, 0.0};
      zero_steer.data = {0.0, 0.0, 0.0, 0.0};

      driving_pub_->publish(zero_drive);
      steering_pub_->publish(zero_steer);

      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    void sendZeroCommands()
    {
        std_msgs::msg::Float64MultiArray zero_drive;
        std_msgs::msg::Float64MultiArray zero_steer;

        zero_drive.data = {0.0, 0.0, 0.0, 0.0};
        zero_steer.data = {0.0, 0.0, 0.0, 0.0};

        driving_pub_->publish(zero_drive);
        steering_pub_->publish(zero_steer);

        RCLCPP_INFO(this->get_logger(), "Published zero commands");
    }

private:
    void desired_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double roll_d, pitch_d, yaw_d;
        quaternionToEuler(msg->pose.pose.orientation, roll_d, pitch_d, yaw_d);
        double x_d = msg->pose.pose.position.x;
        double y_d = msg->pose.pose.position.y;
        double theta_d = yaw_d;

        //velocities are recived transformed in robot base_link frame
        double v_x_d = msg->twist.twist.linear.x;
        double v_y_d = msg->twist.twist.linear.y;
        double omega_d = msg->twist.twist.angular.z;

        //Error transformed in robot frame
        double e_x = (x_d - x_) * cos(theta_) + (y_d - y_) * sin(theta_);
        double e_y = -(x_d - x_) * sin(theta_) + (y_d - y_) * cos(theta_);
        double e_th = angleError(theta_d, theta_);
        double x_w[4] = {this->get_parameter("x_w_1").as_double(), this->get_parameter("x_w_2").as_double(), this->get_parameter("x_w_3").as_double(), this->get_parameter("x_w_4").as_double()};
        double y_w[4] = {this->get_parameter("y_w_1").as_double(), this->get_parameter("y_w_2").as_double(), this->get_parameter("y_w_3").as_double(), this->get_parameter("y_w_4").as_double()};
        double r_w[4] = {this->get_parameter("r_w_1").as_double(), this->get_parameter("r_w_2").as_double(), this->get_parameter("r_w_3").as_double(), this->get_parameter("r_w_4").as_double()};
        Kp_x_ = this->get_parameter("Kp_x").as_double();
        Kp_y_ = this->get_parameter("Kp_y").as_double();
        Kp_th_ = this->get_parameter("Kp_th").as_double();

        double v_x_i[4], v_y_i[4], v_d_i[4], delta_d_i[4];
        double a[4], b[4], omega_c_i[4], delta_c_i[4];
        double max_angular_speed = (210*0.229) * ((2.0*M_PI)/60.0);
        double maxQuotien = 0.0;
        


        for(int i=0;i<4;i++){
          //transform trajectory from cartesian space to robot joint space
          v_x_i[i] = v_x_d - y_w[i] * omega_d;
          v_y_i[i] = v_y_d + x_w[i] * omega_d;
          v_d_i[i] = sqrt(v_x_i[i] * v_x_i[i] + v_y_i[i] * v_y_i[i]);
          delta_d_i[i] = atan2(v_y_i[i], v_x_i[i]);
          
          a[i] = v_d_i[i] * cos(delta_d_i[i]) + Kp_x_ * e_x - Kp_th_ * y_w[i] * e_th;
          b[i] = v_d_i[i] * sin(delta_d_i[i]) + Kp_y_ * e_y + Kp_th_ * x_w[i] * e_th;

          omega_c_i[i] = (sqrt(a[i] * a[i] + b[i] * b[i])) / r_w[i];
          delta_c_i[i] = atan2(b[i], a[i]);
          double quotien=std::abs(omega_c_i[i]/max_angular_speed);
          if (quotien >= maxQuotien)
            maxQuotien = quotien;
        }

        if (maxQuotien > 1.0){
          for(int i=0; i<4; i++){
            omega_c_i[i] /= maxQuotien;
        }
        }
        double returnValueTemp[2];
        std_msgs::msg::Float64MultiArray omega_i_msg;
        std_msgs::msg::Float64MultiArray delta_i_msg;
        for(int i=0;i<4;i++){
          optimiseSpeedAngle(returnValueTemp, omega_c_i[i], delta_c_i[i]);
          omega_i_msg.data.push_back(returnValueTemp[0]);
          delta_i_msg.data.push_back(returnValueTemp[1]);
        }
        RCLCPP_INFO(this->get_logger(), "FL steering angle d: %f", delta_i_msg.data[0]);
        RCLCPP_INFO(this->get_logger(), "BL steering angle d: %f", delta_i_msg.data[1]);
        RCLCPP_INFO(this->get_logger(), "BR steering angle d: %f", delta_i_msg.data[2]);
        RCLCPP_INFO(this->get_logger(), "FR steering angle d: %f", delta_i_msg.data[3]);

        driving_pub_->publish(omega_i_msg);
        steering_pub_->publish(delta_i_msg);
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

    // pub
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr driving_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;

    // state variables
    double x_, y_, theta_;
    double v_x_, v_y_, omega_;
    double Kp_x_, Kp_y_, Kp_th_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ModelBasedController>());
    auto node = std::make_shared<ModelBasedController>();
    rclcpp::on_shutdown([node]() {
        node->sendZeroCommands();
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    });
    return 0;
}
