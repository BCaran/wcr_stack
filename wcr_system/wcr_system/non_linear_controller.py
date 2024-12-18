import rclpy
from rclpy.node import Node
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import rclpy.time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Trigger
import numpy as np


class NonLinearController(Node):
    def __init__(self):
        super().__init__('non_linear_controller')
        self.pose_sub = self.create_subscription(Odometry, '/wcr/odom', self.odometry_callback, 10)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/wcr/joint_cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/wcr/cmd_vel', 10)
        self.pose_desired_pub = self.create_publisher(PoseStamped, "nonlinear_controller/pose_desired", 10)
        self.controller_timer = self.create_timer(1/62.5, self.controller_callback)
        self.start_controller = self.create_service(Trigger, "start_controller", self.start_controller_callback)
        self.stop_controller = self.create_service(Trigger, "stop_controller", self.stop_controller_callback)
        self.declare_parameter("start_using_srv", True)
        self.x_ = 0.0
        self.y_ = 0.0
        self.th_ = 0.0
        self.q_ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_time_ = self.get_clock().now().nanoseconds / 1e9
        self.current_time_ = self.get_clock().now().nanoseconds / 1e9
        self.normalized_time_ = 0.0
        self.q_c_ = Float64MultiArray()
        self.cmd_vel_c = Twist()
        self.x_w_ = [0.1125, -0.1125, -0.1125, 0.1125]
        self.y_w_ = [0.1125, 0.1125, -0.1125, -0.1125]
        self.kp_x_ = 5
        self.kp_y_ = 5
        self.kp_th_ = 2
        self.ki_x_ = 0.0
        self.ki_y_ = 0.0
        self.ki_th_ = 0
        self.e_y_i_ = 0.0
        self.e_x_i_ = 0.0
        self.e_th_i_ = 0.0
        if(self.get_parameter("start_using_srv").get_parameter_value().bool_value):
            self.controller_started = False
        else:
            self.controller_started = True

    def stop_controller_callback(self, request, response):
        response.success = True
        response.message = "Controller stopped"
        self.controller_started = False
        self.cmd_vel_c.linear.x = 0.0
        self.cmd_vel_c.linear.y = 0.0
        self.cmd_vel_c.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_c)
        return response
    
    def start_controller_callback(self, request, response):
        response.success = True
        response.message = "Controller started"
        self.controller_started = True
        self.last_time_ = self.get_clock().now().nanoseconds / 1e9
        self.current_time_ = self.get_clock().now().nanoseconds / 1e9
        return response
        
    def controller_callback(self):
        if(self.controller_started == True and self.normalized_time_ < 40):
            self.current_time_ = self.get_clock().now().nanoseconds / 1e9
            dt = self.current_time_ - self.last_time_
            self.normalized_time_ += dt
            #Reference trajectory
            x_d = 0.0
            y_d = 0.05*self.normalized_time_
            th_d = 0.0

            #Derivative of trajectory
            dot_x_d = 0.0
            dot_y_d = 0.05
            dot_th_d = 0.0

            #Rotated derivative of trajectory
            rot_dot_x_d = dot_x_d * math.cos(th_d) + dot_y_d * math.sin(th_d)
            rot_dot_y_d = -dot_x_d * math.sin(th_d) + dot_y_d * math.cos(th_d)
            dot_x_d = rot_dot_x_d
            dot_y_d = rot_dot_y_d

            #Error
            e_x = (x_d - self.x_)*math.cos(self.th_) + (y_d - self.y_)*math.sin(self.th_)
            e_y = -(x_d - self.x_)*math.sin(self.th_) + (y_d - self.y_)*math.cos(self.th_)
            e_th = th_d - self.th_
            self.e_y_i_ += e_y * dt 
            self.e_x_i_ += e_x * dt
            
            dot_x_i_d = [0.0, 0.0, 0.0, 0.0]
            dot_y_i_d = [0.0, 0.0, 0.0, 0.0]
            dot_xy_d = [0.0, 0.0, 0.0, 0.0]
            delta_i_d = [0.0, 0.0, 0.0, 0.0]
            a = [0, 0, 0, 0]
            b = [0, 0, 0, 0]
            self.q_c_.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            v_c = [0.0, 0.0, 0.0, 0.0]
            delta_c = [0.0, 0.0, 0.0, 0.0]
            W_c = [0.0, 0.0, 0.0, 0.0]
            v_x_c = 0.0
            v_y_c = 0.0
            v_th_c = 0.0
            for i in range(4):
                dot_x_i_d[i] = dot_x_d - self.y_w_[i] * dot_th_d
                dot_y_i_d[i] = dot_y_d + self.x_w_[i] * dot_th_d
                dot_xy_d [i]= math.sqrt(math.pow(dot_x_i_d[i], 2) + math.pow(dot_y_i_d[i], 2))
                delta_i_d[i]= math.atan2(dot_y_i_d[i], dot_x_i_d[i])
                a[i] = dot_xy_d[i] * math.cos(delta_i_d[i]) + self.kp_x_*e_x + self.ki_x_ * self.e_x_i_ - self.kp_th_ * self.y_w_[i]*e_th
                b[i] = dot_xy_d[i] * math.sin(delta_i_d[i]) + self.kp_y_*e_y + self.ki_y_ * self.e_y_i_ + self.kp_th_*self.x_w_[i]*e_th

                v_c[i] = math.sqrt(math.pow(a[i], 2) + math.pow(b[i], 2))
                delta_c[i] = math.atan2(b[i], a[i])

                v_x_c += (math.cos(delta_c[i])/4)*(v_c[i])
                v_y_c += (math.sin(delta_c[i])/4)*(v_c[i])
                W_c[i] = (-self.y_w_[i]*math.cos(delta_c[i]) + self.x_w_[i]*math.sin(delta_c[i]))/(4*math.pow(self.x_w_[i], 2) + 4*math.pow(self.y_w_[i], 2))
                v_th_c +=  W_c[i]*v_c[i]

            #print("e_x:", e_x)
            #print("e_y:", e_y)
            #print("e_th:", e_th)
            #print("e_x_i:", self.e_x_i_)

            #self.joint_cmd_pub.publish(self.q_c_)
            self.cmd_vel_c.linear.x = v_x_c
            self.cmd_vel_c.linear.y = v_y_c
            self.cmd_vel_c.angular.z = v_th_c


            desired_pose = PoseStamped()
            desired_pose.header.stamp = self.get_clock().now().to_msg()
            desired_pose.header.frame_id = "odom"
            desired_pose.pose.position.x = x_d
            desired_pose.pose.position.y = y_d
            quat = quaternion_from_euler(0.0, 0.0, th_d)
            desired_pose.pose.orientation.x = quat[0]
            desired_pose.pose.orientation.y = quat[1]
            desired_pose.pose.orientation.z = quat[2]
            desired_pose.pose.orientation.w = quat[3]

            self.cmd_vel_pub.publish(self.cmd_vel_c)
            self.pose_desired_pub.publish(desired_pose)

            self.last_time_ = self.get_clock().now().nanoseconds / 1e9
        elif (self.normalized_time_ > 40 and self.normalized_time_ < 45):
            self.cmd_vel_c = Twist()
            self.cmd_vel_pub.publish(self.cmd_vel_c)  
        else:
            pass        

    def odometry_callback(self, msg):
        self.x_ = msg.pose.pose.position.x
        self.y_ = msg.pose.pose.position.y
        self.th_ = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.th_  = self.th_ [2]

    #def joint_states_callback(self, msg):
    #    for i in range(4):
    #        self.q_[i] = msg.velocity[i]
    #        self.q_[i+4] = msg.position[i + 4]

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = NonLinearController()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()