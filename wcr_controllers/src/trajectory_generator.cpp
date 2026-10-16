#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node
{
    public:
        TrajectoryPublisher(): Node("trajectory_publisher")
        {
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>("wcr/desired_trajectory", 10);

        //parameters for selecting trajectory
        this->declare_parameter<std::string>("trajectory_type", "circle");
        this->get_parameter("trajectory_type", trajectory_type_);
        
        start_time_ = this->now();
        timer_ = this->create_wall_timer(5ms, std::bind(&TrajectoryPublisher::timer_callback, this));
    }
  private:
    void timer_callback()
    {
        double t = (this->now() - start_time_).seconds();
        double x_d = 0.0, y_d = 0.0, theta_d = 0.0;
        double v_x_d = 0.0, v_y_d = 0.0, omega_d = 0.0;
        double theta_d_unwrapped;

        if (trajectory_type_ == "circle")
        {
            double r = 0.5;
            double v = (2 * M_PI)/40;

            x_d     = r * cos(v * t);
            y_d     = r * sin(v * t);

            v_x_d   = -1 * r * v * sin(v * t);
            v_y_d   = r * v * cos(v * t);

            theta_d = atan2(v_y_d, v_x_d);  // same as v*t
            //theta_d_unwrapped = unwrapAngle(theta_d, theta_prev_, offset_);
            omega_d = v;
        }
        else if (trajectory_type_ == "circle_wall")
        {
            double r = 0.5;
            double v = 0.05;
            double omega = v / r;

            x_d     = r + r * cos(M_PI + omega * t);
            y_d     = r * sin(M_PI + omega * t);

            v_x_d   = -r * omega * sin(M_PI + omega * t);
            v_y_d   = r * omega * cos(M_PI + omega * t);

            theta_d = 0.0;  // same as v*t
            //theta_d_unwrapped = unwrapAngle(theta_d, theta_prev_, offset_);
            omega_d = 0.0;
        }
        else if (trajectory_type_ == "eight")
        {
            const double a = 0.75;
            const double b = 0.75;   // keep your value; you'll get tall lobes (b/2 ≈ 0.375)
            const double v = 0.1;

            const double s  = std::sin(v * t);
            const double c  = std::cos(v * t);
            const double s2 = std::sin(2.0 * v * t);
            const double c2 = std::cos(2.0 * v * t);

            x_d   = a * s;
            y_d   = b * s * c;          // = 0.5*b*sin(2vt)

            v_x_d = a * v * c;
            v_y_d = b * v * c2;

            const double a_x = -a * v * v * s;
            const double a_y = -2.0 * b * v * v * s2;

            theta_d = std::atan2(v_y_d, v_x_d);
            const double sp2 = v_x_d * v_x_d + v_y_d * v_y_d;
            omega_d = (sp2 > 1e-9) ? (v_x_d * a_y - v_y_d * a_x) / sp2 : 0.0;
        }

        else if (trajectory_type_ == "line")
        {
          x_d = 0.05 * t;
          y_d = 0.0;

          v_x_d = 0.05;
          v_y_d = 0.0;

          theta_d = atan2(v_y_d, v_x_d);
          double v_x_dot = 0.0;
          double v_y_dot = 0.0;
          omega_d = 0.0;
        
        }
        else if(trajectory_type_== "flower"){
            double t_f = 140.0;
            double omega = 2*M_PI / t_f;
            x_d = 0.5 * std::cos(2.0 * 2.0 * omega * t) * std::cos(2.0 * omega * t) + 0.1;
            y_d = 0.5 * std::cos(2.0 * 2.0 * omega * t) * std::sin(2.0 * omega * t) + 0.2;
            v_x_d = -0.5 * omega * (4.0 * std::sin(4.0 * omega * t) * std::cos(2.0 * omega * t)
                             + 2.0 * std::cos(4.0 * omega * t) * std::sin(2.0 * omega * t));
            v_y_d = 0.5 * omega * (2.0 * std::cos(4.0 * omega * t) * std::cos(2.0 * omega * t)
                             - 4.0 * std::sin(4.0 * omega * t) * std::sin(2.0 * omega * t));

            double c2 = std::cos(2*omega*t), s2 = std::sin(2*omega*t);
            double c4 = std::cos(4*omega*t), s4 = std::sin(4*omega*t);
            double w2 = omega*omega;

            double a_x_d = -0.5*w2*(16*c4*c2 - 8*s4*s2 - 8*s4*s2 + 4*c4*c2);   // = -0.5*w2*(20*c4*c2 - 16*s4*s2)
            double a_y_d =  0.5*w2*(-8*s4*c2 - 4*c4*s2 - 16*c4*s2 - 8*s4*c2);  // =  0.5*w2*(-16*s4*c2 - 20*c4*s2)

            omega_d = (v_x_d*a_y_d - v_y_d*a_x_d) / (v_x_d*v_x_d + v_y_d*v_y_d);

            theta_d = atan2(v_y_d, v_x_d);
            last_t_ = t;
            last_theta_d_ = theta_d;
        }

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = x_d;
        msg.pose.pose.position.y = y_d;
        geometry_msgs::msg::Quaternion q;
        tf2::Quaternion tf_q;
        tf_q.setRPY(0.0, 0.0, theta_d);
        msg.pose.pose.orientation = tf2::toMsg(tf_q);

        // velocities are in base_link frame
        msg.twist.twist.linear.x = v_x_d * cos(theta_d) + v_y_d * sin(theta_d);
        msg.twist.twist.linear.y = -v_x_d * sin(theta_d) + v_y_d * cos(theta_d);
        msg.twist.twist.angular.z = omega_d;

        pub_->publish(msg);
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
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string trajectory_type_;
    rclcpp::Time start_time_;
    double theta_prev_ = 0.0;
    double offset_ = 0.0;
    double last_theta_d_ = 0.0;
    double last_t_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}