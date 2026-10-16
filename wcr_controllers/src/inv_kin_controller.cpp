#include <memory>
#include <cmath> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class CmdVelSubscriber : public rclcpp::Node
{
  public:
    CmdVelSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "wcr/cmd_vel", 10,
            std::bind(&CmdVelSubscriber::cmdVelCallback, this, std::placeholders::_1));
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
    }

  private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      double x_w[4] = {this->get_parameter("x_w_1").as_double(), this->get_parameter("x_w_2").as_double(), this->get_parameter("x_w_3").as_double(), this->get_parameter("x_w_4").as_double()};
      double y_w[4] = {this->get_parameter("y_w_1").as_double(), this->get_parameter("y_w_2").as_double(), this->get_parameter("y_w_3").as_double(), this->get_parameter("y_w_4").as_double()};
      double r_w[4] = {this->get_parameter("r_w_1").as_double(), this->get_parameter("r_w_2").as_double(), this->get_parameter("r_w_3").as_double(), this->get_parameter("r_w_4").as_double()};
      double v_i;
      double omega_i[4];
      double delta_i[4];
      double max_angular_speed = (210*0.229) * ((2.0*M_PI)/60.0);
      double maxQuotien = 0.0;
      for(int i =0; i<4;i++){
        double v_x_i = msg->linear.x - y_w[i]*msg->angular.z;
        double v_y_i = msg->linear.y + x_w[i]*msg->angular.z;
        v_i = std::sqrt(v_x_i * v_x_i + v_y_i * v_y_i);
        delta_i[i] = std::atan2(v_y_i, v_x_i);
        omega_i[i] = v_i / r_w[i];
        double quotien=std::abs(omega_i[i]/max_angular_speed);
        if (quotien >= maxQuotien)
          maxQuotien = quotien;
      } 
      if (maxQuotien > 1.0){
        for(int i=0; i<4; i++){
          omega_i[i] /= maxQuotien;
        }
      }
      double returnValueTemp[2];
      std_msgs::msg::Float64MultiArray omega_i_msg;
      std_msgs::msg::Float64MultiArray delta_i_msg;
      for(int i=0;i<4;i++){
        optimiseSpeedAngle(returnValueTemp, omega_i[i], delta_i[i]);
        omega_i_msg.data.push_back(returnValueTemp[0]);
        delta_i_msg.data.push_back(returnValueTemp[1]);
      }
      driving_pub_->publish(omega_i_msg);
      steering_pub_->publish(delta_i_msg);
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
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr driving_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSubscriber>());
  rclcpp::shutdown();
  return 0;
}