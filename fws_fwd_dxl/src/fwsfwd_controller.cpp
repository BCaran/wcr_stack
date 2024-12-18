#include <chrono>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#define PI 3.14159265359
#define constrain(amt,low,high) ((amt)<=(low)?(low):((amt)>=(high)?(high):(amt)))
#define PULS_PER_RAD 651.8986469

#define FL_WHEEL 1
#define BL_WHEEL 2
#define BR_WHEEL 3
#define FR_WHEEL 4

#define FL_STEERING 5
#define BL_STEERING 6
#define BR_STEERING 7
#define FR_STEERING 8

using std::placeholders::_1;
using namespace std::chrono_literals;

DynamixelWorkbench dxl_wb;

//Parametri robota
double a_ = 0.225/2;
double b_ = 0.225/2;
double r_ = 0.0254;
double D_ = sqrt((a_ * a_) + (b_ * b_));
double Rx_ = a_ / D_;
double Ry_ = b_ / D_;

std::vector<double> odometry_pose_covariance_;
std::vector<double> odometry_twist_covariance_;


//double x_w_r_[4] = {a_, -a_, -a_, a_};
//double y_w_r_[4] = {b_, b_, -b_, -b_};

//Parametri motora
const double rad_to_value = 651.8986469;
const int32_t angle_offsets[4] = {2048, 2048, 2048, 2048};

uint8_t driving_motors_ids_[4] = {1, 2, 3, 4};
uint8_t steering_motors_ids_[4] = {5, 6, 7, 8};
uint8_t motors_ids_[8] = {1, 2, 3, 4, 5, 6, 7, 8};
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;

class FWSFWDController : public rclcpp::Node
{
  public:
    FWSFWDController()
    : Node("fwsfwd_controller")
    {   

        this->declare_parameter("cmd_vel_topic", "/wcr/cmd_vel");
        this->declare_parameter("joint_state_topic", "/wcr/joint_states");
        this->declare_parameter("odom_topic", "/wcr/odom");      

        this->declare_parameter("dxl_usb_port", "/dev/ttyUSB0");
        this->declare_parameter("dynamixel_limit_max_velocity", 210);
        this->declare_parameter("velocity_ms_constant_value_wheel", (60/(2*PI))/0.229);

        this->declare_parameter("x_w_1", 0.1125);
        this->declare_parameter("y_w_1", 0.1125);
        this->declare_parameter("r_w_1", 0.0254);
        this->declare_parameter("FL_wheel_P", 100);
        this->declare_parameter("FL_wheel_I", 1920);
        this->declare_parameter("FL_steering_P", 800);
        this->declare_parameter("FL_steering_I", 0);
        this->declare_parameter("FL_steering_D", 0);

        this->declare_parameter("x_w_2", -0.1125);
        this->declare_parameter("y_w_2", 0.1125);
        this->declare_parameter("r_w_2", 0.0254);
        this->declare_parameter("BL_wheel_P", 100);
        this->declare_parameter("BL_wheel_I", 1920);
        this->declare_parameter("BL_steering_P", 800);
        this->declare_parameter("BL_steering_I", 0);
        this->declare_parameter("BL_steering_D", 0);

        this->declare_parameter("x_w_3", -0.1125);
        this->declare_parameter("y_w_3", -0.1125);
        this->declare_parameter("r_w_3", 0.0254);
        this->declare_parameter("BR_wheel_P", 100);
        this->declare_parameter("BR_wheel_I", 1920);
        this->declare_parameter("BR_steering_P", 800);
        this->declare_parameter("BR_steering_I", 0);
        this->declare_parameter("BR_steering_D", 0);

        this->declare_parameter("x_w_4", 0.1125);
        this->declare_parameter("y_w_4", -0.1125);
        this->declare_parameter("r_w_4", 0.0254);
        this->declare_parameter("FR_wheel_P", 100);
        this->declare_parameter("FR_wheel_I", 1920);
        this->declare_parameter("FR_steering_P", 800);
        this->declare_parameter("FR_steering_I", 0);
        this->declare_parameter("FR_steering_D", 0);

        this->declare_parameter<std::vector<double>>("odometry_pose_covariance", {0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                    0.0, 0.01, 0.0, 0,0, 0,0, 0,0,
                                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01});

        this->declare_parameter<std::vector<double>>("odometry_twist_covariance", {0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                    0.0, 0.01, 0.0, 0,0, 0,0, 0,0,
                                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01});


        const char * log = nullptr;

        subscription_cmdvel_ = this->create_subscription<geometry_msgs::msg::Twist>(this->get_parameter("cmd_vel_topic").as_string(), 10, std::bind(&FWSFWDController::cmd_vel_callback, this, _1));
        subscription_jointcmd_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("wcr/joint_cmd", 10, std::bind(&FWSFWDController::joint_cmd_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(this->get_parameter("joint_state_topic").as_string(), 10);
        publisher_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(this->get_parameter("odom_topic").as_string(), 10);
        odometry_reset_ = this->create_service<std_srvs::srv::Trigger>("reset_odometry", std::bind(&FWSFWDController::odom_reset, this, std::placeholders::_1, std::placeholders::_2));
        timer_ = this->create_wall_timer(2ms, std::bind(&FWSFWDController::joint_state_callback, this));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "fwsfwd_controller Running");
        if (!dxl_wb.init(this->get_parameter("dxl_usb_port").as_string().c_str(), 3000000, &log)) {
            RCLCPP_INFO(this->get_logger(), "Dynamixels failed to init");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Dynamixels init succesful");
        }
        
        int64_t P_gains[8] = {this->get_parameter("FL_wheel_P").as_int(), this->get_parameter("BL_wheel_P").as_int(), this->get_parameter("BR_wheel_P").as_int(), this->get_parameter("FR_wheel_P").as_int(),
                        this->get_parameter("FL_steering_P").as_int(), this->get_parameter("BL_steering_P").as_int(), this->get_parameter("BR_steering_P").as_int(), this->get_parameter("FR_steering_P").as_int()};
        int64_t I_gains[8] = {this->get_parameter("FL_wheel_I").as_int(), this->get_parameter("BL_wheel_I").as_int(), this->get_parameter("BR_wheel_I").as_int(), this->get_parameter("FR_wheel_I").as_int(),
                        this->get_parameter("FL_steering_I").as_int(), this->get_parameter("BL_steering_I").as_int(), this->get_parameter("BR_steering_I").as_int(), this->get_parameter("FR_steering_I").as_int()};
        int64_t D_gains[4] = {this->get_parameter("FL_steering_D").as_int(), this->get_parameter("BL_steering_D").as_int(), this->get_parameter("BR_steering_D").as_int(), this->get_parameter("FR_steering_D").as_int()};
        odometry_pose_covariance_ = this->get_parameter("odometry_pose_covariance").as_double_array();
        odometry_twist_covariance_ = this->get_parameter("odometry_twist_covariance").as_double_array();

        for (uint i = 1; i < 9; ++i) {
            uint16_t model_number = 0;
            if (!dxl_wb.ping(i, &model_number, &log)) {
                RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] isn't pinged", i);
            }
            if (i < 5 ){
                if(!dxl_wb.setVelocityControlMode(i, &log))
                    RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] didn't change velocity control mode", i);

                if(!dxl_wb.itemWrite(i, "Velocity_P_Gain", P_gains[i-1], &log))
                    RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] didn't change velocity P gain", i);

                if(!dxl_wb.itemWrite(i, "Velocity_I_Gain", I_gains[i-1], &log))
                    RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] didn't change velocity I gain", i);
            }
            else{
                if(!dxl_wb.setPositionControlMode(i, &log))
                    RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] didn't change position control mode", i);

                if(!dxl_wb.itemWrite(i, "Position_P_Gain", P_gains[i-1], &log))
                    RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] didn't change position P gain", i);

                if(!dxl_wb.itemWrite(i, "Position_I_Gain", I_gains[i-1], &log))
                    RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] didn't change position I gain", i);
                
                if(!dxl_wb.itemWrite(i, "Position_D_Gain", D_gains[i - 5], &log))
                    RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] didn't change position D gain", i);

            }
            if (!dxl_wb.torqueOn(i, &log)) {
                RCLCPP_WARN(this->get_logger(), "Dynamixel [%i] torque on error", i);
            }

        }
        //Setting zero position
        int32_t steering_angles[4] = {2048, 2048, 2048, 2048};

        dxl_wb.addSyncWriteHandler(steering_motors_ids_[0], "Goal_Position", &log);
        dxl_wb.addSyncWriteHandler(driving_motors_ids_[0], "Goal_Velocity", &log);

        dxl_wb.syncWrite(kGoalPositionIndex, steering_motors_ids_, 4, steering_angles, 1, &log);
        dxl_wb.addSyncReadHandler(126, 12, &log);
    }


    private:
        float x_ = 0.0;
        float y_ = 0.0;
        float th_ = 0.0;
        rclcpp::Time odom_current_time_;
        rclcpp::Time odom_last_time_;

        void odom_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, const std::shared_ptr<std_srvs::srv::Trigger::Response> response) 
        {
            if (request){ //Avoiding unused_variable warning
                this->x_ = 0.0;
                this->y_ = 0.0;
                this->th_ = 0.0;
                response->success = true;
                response->message = "Odometry reseted";
                RCLCPP_INFO(this->get_logger(), "Odometry reseted");
            }
            
        }

        void joint_cmd_callback(const std_msgs::msg::Float64MultiArray:: SharedPtr msg) const
        {
            bool result = false;
            const char* log = NULL;

            uint8_t velocity_id_array[4] = {FL_WHEEL, FR_WHEEL, BL_WHEEL, BR_WHEEL}; //ID's for each motor that produces speed of robot
            uint8_t position_id_array[4] = {FL_STEERING, FR_STEERING, BL_STEERING, BR_STEERING}; //ID's for each motor that rotates speed motor

            double velocity_constant_value = 1 / (0.229 * 0.10472 * r_);

            int32_t dynamixel_velocity[4]; //array for motor speed values
            int32_t dynamixel_position[4]; //array for motor angle values
            double velocity_array_m_s[4] = {0.0, 0.0, 0.0, 0.0}; //translational speed each motor need's to produce
            double steering_array[4] = {0.0, 0.0, 0.0, 0.0}; //angle that each motor need's to make

            for (int i = 0;i<4;i++){
                velocity_array_m_s[i] = msg->data[i];
                steering_array[i] = msg->data[i+4];
            }

            for(int i = 0;i < 4;i++)
                dynamixel_position[i] = 2048 + steering_array[i] * PULS_PER_RAD;

            for(int i = 0;i < 4;i++)
                dynamixel_velocity[i] = velocity_array_m_s[i] * velocity_constant_value;

            result = dxl_wb.syncWrite(kGoalPositionIndex, position_id_array, 4, dynamixel_position  , 1, &log);
            if (result == false){
                RCLCPP_INFO(this->get_logger(), "%s", log);
            }

            result = dxl_wb.syncWrite(kGoalVelocityIndex, velocity_id_array, 4, dynamixel_velocity  , 1, &log);
            if (result == false){
               RCLCPP_INFO(this->get_logger(), "%s", log);
            }
        }

        void optimiseSpeedAngle(double returnSpeedAngle[], double speed, double angle) const{ 
            if(angle > (2*PI/3)){
                returnSpeedAngle[0] = -1 * speed;
                returnSpeedAngle[1] = angle - PI;
            }
            else if (angle <= (-2*PI/3)){
                returnSpeedAngle[0] = -1 * speed;
                returnSpeedAngle[1] = angle + PI;
            }
            else{
                returnSpeedAngle[0] = speed;
                returnSpeedAngle[1] = angle; 
            }
        }

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
        {
            bool result = false;
            const char* log = NULL;

            uint8_t velocity_id_array[4] = {FL_WHEEL, FR_WHEEL, BL_WHEEL, BR_WHEEL}; //ID's for each motor that produces speed of robot
            uint8_t position_id_array[4] = {FL_STEERING, FR_STEERING, BL_STEERING, BR_STEERING}; //ID's for each motor that rotates speed motor
            int32_t dynamixel_velocity[4]; //array for motor speed values
            int32_t dynamixel_position[4]; //array for motor angle values
            double velocity_array_m_s[4] = {0.0, 0.0, 0.0, 0.0}; //translational speed each motor need's to produce
            double steering_array[4] = {0.0, 0.0, 0.0, 0.0}; //angle that each motor need's to make

            double x_w[4] = {this->get_parameter("x_w_1").as_double(), this->get_parameter("x_w_2").as_double(), this->get_parameter("x_w_3").as_double(), this->get_parameter("x_w_4").as_double()};
            double y_w[4] = {this->get_parameter("y_w_1").as_double(), this->get_parameter("y_w_2").as_double(), this->get_parameter("y_w_3").as_double(), this->get_parameter("y_w_4").as_double()};
            double r_w[4] = {this->get_parameter("r_w_1").as_double(), this->get_parameter("r_w_2").as_double(), this->get_parameter("r_w_3").as_double(), this->get_parameter("r_w_4").as_double()};

            double robotDimMatrix[8][3] = {{1, 0, -1 * y_w[0]}, {0, 1, x_w[0]}, {1, 0, -1 * y_w[3]}, {0, 1, x_w[3]}, {1, 0, -1 * y_w[1]}, {0, 1, x_w[1]}, {1, 0, -1 * y_w[2]}, {0, 1, x_w[2]}};
            double inputRobotSpeeds[3] = {msg->linear.x, msg->linear.y, msg->angular.z};
            double outputSpeedsVxVy[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //needed speed for each motor

            //Množenje matrica dimenzija robota i ulaznih brzina kako bi se dobile izlazne brzine
            for(int i = 0; i < 8; i++)
                for(int j = 0; j < 3; j++)
                outputSpeedsVxVy[i] += robotDimMatrix[i][j]*inputRobotSpeeds[j];

            double maxQuotien = 0.0;
            for(int i = 0; i < 4; i++){
                velocity_array_m_s[i] = sqrt((outputSpeedsVxVy[2*i] * outputSpeedsVxVy[2*i]) + (outputSpeedsVxVy[2*i+1] * outputSpeedsVxVy[2*i+1]));
                steering_array[i] = atan2(outputSpeedsVxVy[2*i+1], outputSpeedsVxVy[2*i]);
                double max_linear_speed = this->get_parameter("dynamixel_limit_max_velocity").as_int()*r_w[i]/this->get_parameter("velocity_ms_constant_value_wheel").as_double();
                double quotien=std::abs(velocity_array_m_s[i]/max_linear_speed);
                if ( quotien >= maxQuotien)
                    maxQuotien = quotien;
            }
            if (maxQuotien > 1.0){
                for(int i = 0; i < 4; i++){
                    velocity_array_m_s[i] /= maxQuotien;
                }
             }

            double returnValueTemp[2];
            for(int i = 0;i < 4;i++){
                optimiseSpeedAngle(returnValueTemp, velocity_array_m_s[i], steering_array[i]);
                velocity_array_m_s[i] = returnValueTemp[0];
                steering_array[i] = returnValueTemp[1];
            }

            for(int i = 0;i < 4;i++)
                dynamixel_position[i] = 2048 + steering_array[i] * PULS_PER_RAD;

            for(int i = 0;i < 4;i++)
                dynamixel_velocity[i] = constrain(velocity_array_m_s[i] / r_w[i] * this->get_parameter("velocity_ms_constant_value_wheel").as_double() , -this->get_parameter("dynamixel_limit_max_velocity").as_int(), this->get_parameter("dynamixel_limit_max_velocity").as_int());

            result = dxl_wb.syncWrite(kGoalPositionIndex, position_id_array, 4, dynamixel_position  , 1, &log);
            if (result == false){
                RCLCPP_INFO(this->get_logger(), "%s", log);
            }

            result = dxl_wb.syncWrite(kGoalVelocityIndex, velocity_id_array, 4, dynamixel_velocity  , 1, &log);
            if (result == false){
               RCLCPP_INFO(this->get_logger(), "%s", log);
            }
        }

        void joint_state_callback()
        {

            int32_t positions[8];
            int32_t velocities[8];
            int32_t currents[8];
            const char* log = NULL;
            dxl_wb.syncRead(kPresentPositionVelocityCurrentIndex, motors_ids_, 8, &log);
            dxl_wb.getSyncReadData(kPresentPositionVelocityCurrentIndex, motors_ids_, 8, 126, 2, currents, &log);
            dxl_wb.getSyncReadData(kPresentPositionVelocityCurrentIndex, motors_ids_, 8, 128, 4, velocities, &log);
            dxl_wb.getSyncReadData(kPresentPositionVelocityCurrentIndex, motors_ids_, 8, 132, 4, positions, &log);
            auto joint_states = sensor_msgs::msg::JointState();
            auto odometry = nav_msgs::msg::Odometry();
            geometry_msgs::msg::TransformStamped t;
            joint_states.header.stamp = this->get_clock()->now();
            joint_states.name = {"FL_wheel", "BL_wheel", "BR_wheel", "FR_wheel", "FL_steering", "BL_steering", "BR_steering", "FR_steering"};
            joint_states.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            joint_states.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            joint_states.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            for(int i = 0; i < 8;i++){
                joint_states.position[i] = dxl_wb.convertValue2Radian(motors_ids_[i], positions[i]);
                joint_states.velocity[i] = dxl_wb.convertValue2Velocity(motors_ids_[i], velocities[i]);
                joint_states.effort[i] = dxl_wb.convertValue2Current(motors_ids_[i], currents[i]);
            }
            publisher_->publish(joint_states);

            //dio koda za slanje odometrije
            double v_x = 0.0;
            double v_y = 0.0;
            double omega = 0.0;
            float a = 0.225/2;
            float b = 0.225/2;
            float r = 0.0254;
            float x_w_r[4] = {a, -a, -a, a};
            float y_w_r[4] = {b, b, -b, -b};
            float W[4];
            float q[8];
            for (int i = 0;i<4;i++){
                q[i] = joint_states.velocity[i];
                q[i + 4] = joint_states.position[i+4];
            }

            for(int i =0;i<4;i++){
                v_x = v_x + (cos(q[i+4])/4)*(q[i]*r);
                v_y = v_y + (sin(q[i+4])/4)*(q[i]*r);
                W[i] = (-y_w_r[i]*cos(q[i+4]) + x_w_r[i]*sin(q[i+4]))/(4*pow(x_w_r[i], 2) + 4*pow(y_w_r[i], 2));
                omega = omega + W[i]*q[i]*r;
	        }
            this->odom_current_time_ = this->get_clock()->now();
            float dt = this->odom_current_time_.seconds() - this->odom_last_time_.seconds();

            this->th_ += omega*dt;
            this->x_ += (v_x * cos(this->th_) - v_y * sin(this->th_)) * dt;
            this->y_ += (v_x * sin(this->th_) + v_y * cos(this->th_)) * dt;

            this->odom_last_time_ = this->get_clock()->now();

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, this->th_);

            odometry.header.stamp = this->get_clock()->now();
            odometry.header.frame_id = "odom";
            odometry.child_frame_id = "base_link";
            //Pose
            odometry.pose.pose.position.x = this->x_;
            odometry.pose.pose.position.y = this->y_;
            odometry.pose.pose.position.z = 0.0;
            odometry.pose.pose.orientation.x = quat.x();
            odometry.pose.pose.orientation.y = quat.y();
            odometry.pose.pose.orientation.z = quat.z();
            odometry.pose.pose.orientation.w = quat.w();

            //Pose covariance
            for(int i = 0; i<36; i++){
                odometry.pose.covariance[i] = odometry_pose_covariance_[i];
                odometry.twist.covariance[i] = odometry_twist_covariance_[i];
            }
            odometry.twist.twist.linear.x = v_x;
            odometry.twist.twist.linear.y = v_y;
            odometry.twist.twist.linear.z = 0.0;
            odometry.twist.twist.angular.x = 0.0;
            odometry.twist.twist.angular.y = 0.0;
            odometry.twist.twist.angular.z = omega;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = this->x_;
            t.transform.translation.y = this->y_;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = quat.x();
            t.transform.rotation.y = quat.y();
            t.transform.rotation.z = quat.z();
            t.transform.rotation.w = quat.w();

            tf_broadcaster_->sendTransform(t);
            publisher_odom_->publish(odometry);

        }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;
    size_t count_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmdvel_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_jointcmd_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr odometry_reset_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FWSFWDController>());
  rclcpp::shutdown();
  return 0;
}
