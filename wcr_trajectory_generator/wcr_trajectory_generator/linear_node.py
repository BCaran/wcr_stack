import rclpy
from rclpy.node import Node
from wcr_interfaces.msg import DesiredPoseTwist
import numpy as np

use_quintic_scaling = False

line_angle = np.deg2rad(90.0)
line_length = 1.0
vx_amplitude = 0.05
vy_amplitude = 0.05
sampling_rate = 1000  # Samples per second

class LinearTrajectoryNode(Node):

    def __init__(self):
        super().__init__('linear_trajectory')
        self.publisher_ = self.create_publisher(DesiredPoseTwist, '/wcr/desired_pose_twist', 10)
        self.timer = self.create_timer(1/sampling_rate, self.trajectory_callback)
        self.desired_pose_twist_msg = DesiredPoseTwist()

        self.vx = vx_amplitude * np.cos(line_angle)
        self.vy = vy_amplitude * np.sin(line_angle)
        speed = np.sqrt(self.vx**2 + self.vy**2)
        self.total_time = line_length/speed

        self.start_time = self.get_clock().now().nanoseconds
        self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9

    def trajectory_callback(self):
        if self.current_time_s <= self.total_time:
            self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
            self.desired_pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.desired_pose_twist_msg.header.frame_id = "odom"

            #using linear time scaling
            if use_quintic_scaling == False:
                self.desired_pose_twist_msg.pose.position.x = self.vx*self.current_time_s
                self.desired_pose_twist_msg.pose.position.y = self.vy*self.current_time_s
                
                self.desired_pose_twist_msg.twist.linear.x = self.vx
                self.desired_pose_twist_msg.twist.linear.y = self.vy

                self.publisher_.publish(self.desired_pose_twist_msg)

            #using quintic time scaling
            else:
                t = self.current_time_s
                T = self.total_time
                self.desired_pose_twist_msg.pose.position.x = 10 * (t/T)**3 - 15 * (t/T)**4 + 6 * (t/T)**5
                self.desired_pose_twist_msg.pose.position.y = 0.0

                self.desired_pose_twist_msg.twist.linear.x = (30 * (t/T)**2 - 60 * (t/T)**3 + 30 * (t/T)**4) / T
                self.desired_pose_twist_msg.twist.linear.y = 0.0

                self.publisher_.publish(self.desired_pose_twist_msg)

        else:
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = 0.0
            self.publisher_.publish(self.desired_pose_twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LinearTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()