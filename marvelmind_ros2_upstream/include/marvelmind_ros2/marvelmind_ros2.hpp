/*
Copyright (c) 2022, Marvelmind Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUfTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

// C++ STL
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#ifndef WIN32
#include <semaphore.h>
#include <fcntl.h>
#include <thread>
#endif
using namespace std::chrono_literals;

extern "C" 
{
    #include "marvelmind_hedge.h"
}

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

// Needed external messages 
#include "marvelmind_ros2_msgs/msg/hedge_position.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position_addressed.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position_angle.hpp"

#include "marvelmind_ros2_msgs/msg/hedge_imu_raw.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_imu_fusion.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_telemetry.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_quality.hpp"

#include "marvelmind_ros2_msgs/msg/beacon_distance.hpp"
#include "marvelmind_ros2_msgs/msg/beacon_position_addressed.hpp"

#include "marvelmind_ros2_msgs/msg/marvelmind_waypoint.hpp"

#include "marvelmind_ros2_msgs/msg/marvelmind_user_data.hpp"


// global vars
// global semaphore to avoid class scope issues 
#ifndef WIN32
static sem_t *sem;
#else
//static volatile uint8_t sem_int = 0;
HANDLE sem;
#endif

class marvelmind_ros2 : public rclcpp::Node
{
    public:
        marvelmind_ros2();
        ~marvelmind_ros2();
    private:

        // ROS2 Setup
        // Publishers
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgePositionAngle>::SharedPtr hedge_pos_ang_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgePositionAddressed>::SharedPtr hedge_pos_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgePosition>::SharedPtr hedge_pos_noaddress_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::BeaconPositionAddressed>::SharedPtr beacons_pos_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgeImuRaw>::SharedPtr hedge_imu_raw_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgeImuFusion>::SharedPtr hedge_imu_fusion_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::BeaconDistance>::SharedPtr beacon_distance_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgeTelemetry>::SharedPtr hedge_telemetry_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgeQuality>::SharedPtr hedge_quality_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::MarvelmindWaypoint>::SharedPtr marvelmind_waypoint_publisher;
        rclcpp::Publisher<marvelmind_ros2_msgs::msg::MarvelmindUserData>::SharedPtr marvelmind_user_data_publisher;

        // Timer to execute publications so we can easily control rate
        rclcpp::TimerBase::SharedPtr marvelmind_ros2_pub_timer;

        // Function prototypes
        int hedgeReceivePrepare();
        bool hedgeReceiveCheck(void);
        bool beaconReceiveCheck(void);
        bool hedgeIMURawReceiveCheck(void);
        bool hedgeIMUFusionReceiveCheck(void);
        void getRawDistance(uint8_t index);
        bool hedgeTelemetryUpdateCheck(void);
        bool hedgeQualityUpdateCheck(void);  
        bool marvelmindWaypointUpdateCheck(void);
        bool marvelmindUserDataUpdateCheck(void);
        void publishTimerCallback();


        // Variables
        // Topic names
        std::string hedgehog_pos_topic;
        std::string hedgehog_pos_addressed_topic;
        std::string hedgehog_pos_angle_topic;
        std::string hedgehog_imu_raw_topic;
        std::string hedgehog_imu_fusion_topic;
        std::string hedgehog_telemetry_topic;
        std::string hedgehog_quality_topic;
        std::string beacon_raw_distance_topic;
        std::string beacon_pos_addressed_topic;
        std::string marvelmind_waypoint_topic;
        std::string marvelmind_user_data_topic;

        // Config variables
        std::string data_input_semaphore_name;
        #ifdef WIN32
        int tty_baudrate;
        #else
        uint tty_baudrate;
        #endif
        std::string tty_filename;
        int publish_rate_in_hz;
        std::chrono::milliseconds publish_rate_ms;

        // Marvelmind data
        struct MarvelmindHedge * hedge = NULL;
        int64_t hedge_timestamp_prev = 0;
        struct timespec ts;
        uint8_t beaconReadIterations;

        // setup empty messages for sending
        marvelmind_ros2_msgs::msg::HedgePosition hedge_pos_noaddress_msg;// hedge coordinates message (old version without address) for publishing to ROS topic
        marvelmind_ros2_msgs::msg::HedgePositionAddressed hedge_pos_msg;// hedge coordinates message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::HedgePositionAngle hedge_pos_ang_msg;// hedge coordinates and angle message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::BeaconPositionAddressed beacon_pos_msg;// stationary beacon coordinates message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::HedgeImuRaw hedge_imu_raw_msg;// raw IMU data message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::HedgeImuFusion hedge_imu_fusion_msg;// IMU fusion data message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::BeaconDistance beacon_raw_distance_msg;// Raw distance message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::HedgeTelemetry hedge_telemetry_msg;// Telemetry message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::HedgeQuality hedge_quality_msg;// Quality message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::MarvelmindWaypoint marvelmind_waypoint_msg;// Waypoint message for publishing to ROS topic
        marvelmind_ros2_msgs::msg::MarvelmindUserData marvelmind_user_data_msg;// Waypoint message for publishing to ROS topic

};





