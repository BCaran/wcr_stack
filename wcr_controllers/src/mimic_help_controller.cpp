#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"

class JointStateSubscriber : public rclcpp::Node
{
public:
  JointStateSubscriber()
  : Node("mimic_help_controller")
  {
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&JointStateSubscriber::jointStateCallback, this, std::placeholders::_1)
    );

    mimic_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/mimic_driving_controller/commands", 10);

    mimic_steering_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/mimic_steering_controller/commands", 10);
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (joint_index_.empty()) {
          for (size_t i = 0; i < msg->name.size(); ++i) {
              joint_index_[msg->name[i]] = i;
          }
    }
    double v = msg->velocity[joint_index_["FL_wheel"]];
    double delta_f = msg->position[joint_index_["FL_steering"]];
    double delta_r = msg->position[joint_index_["BL_steering"]];

    std_msgs::msg::Float64MultiArray mimic_velocity_msg;
    std_msgs::msg::Float64MultiArray mimic_steering_msg;

    for (int i=0;i<3;i++){
        mimic_velocity_msg.data.push_back(v);
    }

    mimic_steering_msg.data.push_back(delta_f);
    mimic_steering_msg.data.push_back(delta_r);

    mimic_velocity_pub_->publish(mimic_velocity_msg);
    mimic_steering_pub_->publish(mimic_steering_msg);

  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  std::unordered_map<std::string, size_t> joint_index_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mimic_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mimic_steering_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateSubscriber>());
  rclcpp::shutdown();
  return 0;
}
