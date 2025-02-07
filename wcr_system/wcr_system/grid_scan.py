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
from std_srvs.srv import Trigger

def compute_motion_time(distance, v_max):
    peak_velocity_factor = 1.875  # Maximum of ds_quintic at peak
    T = (distance / v_max) * peak_velocity_factor
    return abs(T)

#Square trajectory parameters:
x_amplitude = 1.0
dy_amplitude = 0.1
vx_amplitude = 0.1
vy_amplitude = 0.025

# Parameters for the custom wave pattern (x-t)
x_duration = compute_motion_time(x_amplitude, vx_amplitude)  # Duration of positive phase for x-t (seconds)
y_duration = compute_motion_time(dy_amplitude, vy_amplitude)
pause_duration = 2  # Duration of pause for both pauses in x-t (seconds)
repeats = 3  # Number of repetitions
sampling_rate = 1000  # Samples per second

# Total duration of one cycle (positive, first pause, negative, second pause)
cycle_duration = x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration + pause_duration
total_duration = cycle_duration * repeats


class GridScan(Node):
    def __init__(self):
        super().__init__('non_linear_controller')
        self.current_pose_sub = self.create_subscription(Odometry, '/wcr/odom', self.odometry_callback, 10)
        self.desired_pose_twist_pub = self.create_publisher(DesiredPoseTwist, '/wcr/desired_pose_twist', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/wcr/cmd_vel', 10)
        self.cli = self.create_client(Trigger, '/wcr/reset_odometry')
        self.req = Trigger.Request()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        

        controller_time = self.create_timer(1/50, self.controller_callback)

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

        self.q_c_ = Float64MultiArray()
        self.cmd_vel_c = Twist()
        self.x_w_ = [0.1125, -0.1125, -0.1125, 0.1125]
        self.y_w_ = [0.1125, 0.1125, -0.1125, -0.1125]
        self.kp_x_ = 5
        self.kp_y_ = 5
        self.kp_th_ = 2
        self.e_y_i_ = 0.0
        self.e_x_i_ = 0.0
        self.e_th_i_ = 0.0

        self.dy_repeats = 0
        self.last_dy_repeats = 0
        self.start_time = self.get_clock().now().nanoseconds
        
    def controller_callback(self):
        if self.dy_repeats < repeats:
            self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
            cycle_time = self.current_time_s % cycle_duration  # Time within the current cycle
            #Ravno po X
            if 0 < cycle_time <= x_duration:
                T = x_duration
                t = cycle_time
                s_quintic = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
                ds_quintic = (30 * (t/T)**2 - 60 * (t/T)**3 + 30 * (t/T)**4) / T

                self.x_d_ = s_quintic * x_amplitude
                self.vx_d_ = ds_quintic * x_amplitude                

                vx_c, vy_c, omega_c = self.calculate_controlled_cmd(x_d=self.x_d_, y_d=self.y_d_, th_d=self.th_d_, vx_d=self.vx_d_, vy_d=self.vy_d_, omega_d=self.omega_d_)
                
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = vx_c
                cmd_vel_msg.linear.y = vy_c
                cmd_vel_msg.angular.z = omega_c
                self.cmd_vel_pub.publish(cmd_vel_msg)

                self.last_dy_repeats = self.dy_repeats
            #Pauza 
            elif x_duration < cycle_time <= x_duration + pause_duration:
                self.vx_d_ = 0.0
                self.vy_d_ = 0.0

                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.linear.y = vy_amplitude * 1e-5
                cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel_msg)
            #Po y osi
            elif x_duration + pause_duration < cycle_time <= x_duration + pause_duration + y_duration:
                T = y_duration
                t = cycle_time - (x_duration + pause_duration)
                s_quintic = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
                ds_quintic = (30 * (t/T)**2 - 60 * (t/T)**3 + 30 * (t/T)**4) / T

                self.y_d_ = self.dy_repeats*dy_amplitude + dy_amplitude*s_quintic
                self.vx_d_ = 0.0
                self.vy_d_ = ds_quintic*dy_amplitude

                vx_c, vy_c, omega_c = self.calculate_controlled_cmd(x_d=self.x_d_, y_d=self.y_d_, th_d=self.th_d_, vx_d=self.vx_d_, vy_d=self.vy_d_, omega_d=self.omega_d_)
                
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = vx_c
                cmd_vel_msg.linear.y = vy_c
                cmd_vel_msg.angular.z = omega_c
                self.cmd_vel_pub.publish(cmd_vel_msg)
            #Pauza
            elif x_duration + pause_duration + y_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration:
                self.vx_d_ = 0.0
                self.vy_d_ = 0.0

                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel_msg)
                if(self.dy_repeats == self.last_dy_repeats):
                    self.dy_repeats += 1.0
            #Unazad po X osi
            elif x_duration + pause_duration + y_duration + pause_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration + x_duration:
                T = x_duration
                t = cycle_time - (x_duration + pause_duration + y_duration + pause_duration)

                s_quintic = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
                ds_quintic = (30 * (t/T)**2 - 60 * (t/T)**3 + 30 * (t/T)**4) / T

                self.x_d_ = x_amplitude - x_amplitude*s_quintic
                self.vx_d_= -ds_quintic*x_amplitude
                self.vy_d_ = 0.0
                self.last_dy_repeats = self.dy_repeats

                vx_c, vy_c, omega_c = self.calculate_controlled_cmd(x_d=self.x_d_, y_d=self.y_d_, th_d=self.th_d_, vx_d=self.vx_d_, vy_d=self.vy_d_, omega_d=self.omega_d_)
                
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = vx_c
                cmd_vel_msg.linear.y = vy_c
                cmd_vel_msg.angular.z = omega_c
                self.cmd_vel_pub.publish(cmd_vel_msg)
            #Pauza 1 sekundu
            elif x_duration + pause_duration + y_duration + pause_duration + x_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration:
                self.vx_d_ = 0.0
                self.vy_d_ = 0.0
                
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.linear.y = vy_amplitude * 1e-5
                cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel_msg)
            #Po Y osi
            elif x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration:
                T = y_duration
                t = cycle_time - (x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration)
                s_quintic = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
                ds_quintic = (30 * (t/T)**2 - 60 * (t/T)**3 + 30 * (t/T)**4) / T

                self.y_d_ = self.dy_repeats*dy_amplitude + dy_amplitude*s_quintic
                self.vx_d_ = 0.0
                self.vy_d_ = ds_quintic*dy_amplitude
                vx_c, vy_c, omega_c = self.calculate_controlled_cmd(x_d=self.x_d_, y_d=self.y_d_, th_d=self.th_d_, vx_d=self.vx_d_, vy_d=self.vy_d_, omega_d=self.omega_d_)
                
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = vx_c
                cmd_vel_msg.linear.y = vy_c
                cmd_vel_msg.angular.z = omega_c
                self.cmd_vel_pub.publish(cmd_vel_msg)

            #Pauza 1 sekundu
            elif x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration + pause_duration:
                self.vx_d_ = 0.0
                self.vy_d_ = 0.0
                
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel_msg)
                if(self.dy_repeats == self.last_dy_repeats):
                    self.dy_repeats += 1.0
        else:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel_msg)

        desired_pose_twist_msg = DesiredPoseTwist()
        desired_pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
        desired_pose_twist_msg.header.frame_id = "odom"
        
        desired_pose_twist_msg.pose.position.x = self.x_d_
        desired_pose_twist_msg.pose.position.y = self.y_d_
        desired_pose_twist_msg.pose.orientation.z = 0.0

        desired_pose_twist_msg.twist.linear.x = self.vx_d_
        desired_pose_twist_msg.twist.linear.y = self.vy_d_
        desired_pose_twist_msg.twist.angular.z = self.omega_d_

        self.desired_pose_twist_pub.publish(desired_pose_twist_msg)
        

    def calculate_controlled_cmd(self, x_d, y_d, th_d, vx_d, vy_d, omega_d):
        #Rotated derivative of trajectory
        rot_vx_d = vx_d * math.cos(th_d) + vy_d * math.sin(th_d)
        rot_vy_d = -vx_d * math.sin(th_d) + vy_d * math.cos(th_d)
        vx_d = rot_vx_d
        vy_d = rot_vy_d

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
            dot_x_i_d[i] = vx_d - self.y_w_[i] * omega_d
            dot_y_i_d[i] = vy_d + self.x_w_[i] * omega_d
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

        return v_x_c, v_y_c, v_th_c

    def odometry_callback(self, msg):
        self.x_ = msg.pose.pose.position.x
        self.y_ = msg.pose.pose.position.y
        self.th_ = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.th_  = self.th_ [2]

def main(args=None):
    rclpy.init(args=args)

    grid_scan = GridScan()

    rclpy.spin(grid_scan)

    grid_scan.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()