import rclpy
from rclpy.node import Node
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import rclpy.time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from wcr_interfaces.msg import DesiredPoseTwist


class NonLinearController(Node):
    def __init__(self):
        super().__init__('non_linear_controller')
        self.pose_sub = self.create_subscription(Odometry, '/wcr/odom', self.odometry_callback, 10)
        self.desired_pose_twist_sub = self.create_subscription(DesiredPoseTwist, '/wcr/desired_pose_twist', self.controller_callback, 10)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/wcr/joint_cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/wcr/cmd_vel', 10)
        self.declare_parameter("start_using_srv", False)
        self.x_d_ = 0.0
        self.y_d_ = 0.0
        self.th_d_ = 0.0
        self.vx_d_ = 0.0
        self.vy_d_ = 0.0
        self.omega_d_ = 0.0
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
        self.kp_x_ = 2
        self.kp_y_ = 2
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
        
    def controller_callback(self, desired_msg):
        self.current_time_ = self.get_clock().now().nanoseconds / 1e9
        dt = self.current_time_ - self.last_time_
        self.normalized_time_ += dt
        #Reference trajectory
        x_d = desired_msg.pose.position.x
        y_d = desired_msg.pose.position.y
        th_d = euler_from_quaternion([desired_msg.pose.orientation.x, desired_msg.pose.orientation.y, desired_msg.pose.orientation.z, desired_msg.pose.orientation.w])
        th_d  = th_d[2]

        #Derivative of trajectory
        dot_x_d = desired_msg.twist.linear.x
        dot_y_d = desired_msg.twist.linear.y
        dot_th_d = desired_msg.twist.angular.z

        #Rotated derivative of trajectory
        rot_dot_x_d = dot_x_d * math.cos(th_d) + dot_y_d * math.sin(th_d)
        rot_dot_y_d = -dot_x_d * math.sin(th_d) + dot_y_d * math.cos(th_d)
        dot_x_d = rot_dot_x_d
        dot_y_d = rot_dot_y_d

        #Error
        e_x = (x_d - self.x_)*math.cos(self.th_) + (y_d - self.y_)*math.sin(self.th_)
        e_y = -(x_d - self.x_)*math.sin(self.th_) + (y_d - self.y_)*math.cos(self.th_)
        e_th = th_d - self.th_
        
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
            a[i] = dot_xy_d[i] * math.cos(delta_i_d[i]) + self.kp_x_*e_x - self.kp_th_ * self.y_w_[i]*e_th
            b[i] = dot_xy_d[i] * math.sin(delta_i_d[i]) + self.kp_y_*e_y + self.kp_th_*self.x_w_[i]*e_th

            v_c[i] = math.sqrt(math.pow(a[i], 2) + math.pow(b[i], 2))
            delta_c[i] = math.atan2(b[i], a[i])

            v_x_c += (math.cos(delta_c[i])/4)*(v_c[i])
            v_y_c += (math.sin(delta_c[i])/4)*(v_c[i])
            W_c[i] = (-self.y_w_[i]*math.cos(delta_c[i]) + self.x_w_[i]*math.sin(delta_c[i]))/(4*math.pow(self.x_w_[i], 2) + 4*math.pow(self.y_w_[i], 2))
            v_th_c +=  W_c[i]*v_c[i]
        self.cmd_vel_c.linear.x = v_x_c
        self.cmd_vel_c.linear.y = v_y_c
        self.cmd_vel_c.angular.z = v_th_c

        self.cmd_vel_pub.publish(self.cmd_vel_c)

        self.last_time_ = self.get_clock().now().nanoseconds / 1e9       

    def odometry_callback(self, msg):
        self.x_ = msg.pose.pose.position.x
        self.y_ = msg.pose.pose.position.y
        self.th_ = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.th_  = self.th_ [2]

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = NonLinearController()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()