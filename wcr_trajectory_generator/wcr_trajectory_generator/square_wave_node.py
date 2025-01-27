import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from scipy import signal
import numpy as np
from wcr_interfaces.msg import DesiredPoseTwist
import math

x_amplitude = 1.0
dy_amplitude = 0.1
vx_amplitude = 0.05
vy_amplitude = 0.05

# Parameters for the custom wave pattern (x-t)
x_duration = x_amplitude/vx_amplitude  # Duration of positive phase for x-t (seconds)
y_duration = dy_amplitude/vy_amplitude
pause_duration = 1  # Duration of pause for both pauses in x-t (seconds)
repeats = 6  # Number of repetitions
sampling_rate = 100  # Samples per second

# Total duration of one cycle (positive, first pause, negative, second pause)
cycle_duration = x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration + pause_duration
total_duration = cycle_duration * repeats

class SquareWaveTrajectoryNode(Node):
    def __init__(self):
        super().__init__('square_wave_trajectory')

        # Publishers
        self.pose_twist_publisher = self.create_publisher(DesiredPoseTwist, 'desired_pose_twist', 10)
        
        self.timer = self.create_timer(0.001, self.trajectory_callback)

        self.start_time = self.get_clock().now().nanoseconds
        self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
        self.last_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9

        self.dy_repeats = 0.0
        self.last_dy_repeats = 0.0

        self.desired_pose_twist_msg = DesiredPoseTwist()

    def trajectory_callback(self):
        self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
        dt = self.current_time_s - self.last_time_s
        self.desired_pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.desired_pose_twist_msg.header.frame_id = "base_link"
        cycle_time = self.current_time_s % cycle_duration  # Time within the current cycle
        #Ravno po X
        if 0 < cycle_time <= x_duration:
            self.desired_pose_twist_msg.pose.position.x = vx_amplitude * cycle_time
            self.desired_pose_twist_msg.twist.linear.x = vx_amplitude
            self.desired_pose_twist_msg.twist.linear.y = 0.0
            self.last_dy_repeats = self.dy_repeats
        #Pauza 1 sekundu
        elif x_duration < cycle_time <= x_duration + pause_duration:
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = 0.0
        #Po y osi
        elif x_duration + pause_duration < cycle_time <= x_duration + pause_duration + y_duration:
            self.desired_pose_twist_msg.pose.position.y = self.dy_repeats*dy_amplitude + vy_amplitude * (cycle_time - x_duration - pause_duration)
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = vy_amplitude
        #Pauza 1 sekundu
        elif x_duration + pause_duration + y_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration:
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = 0.0
            if(self.dy_repeats == self.last_dy_repeats):
                self.dy_repeats += 1.0
        #Unazad po X osi
        elif x_duration + pause_duration + y_duration + pause_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration + x_duration:
            self.desired_pose_twist_msg.pose.position.x = x_amplitude - vx_amplitude * (cycle_time - x_duration - pause_duration - y_duration - pause_duration)
            self.desired_pose_twist_msg.twist.linear.x = -vx_amplitude
            self.desired_pose_twist_msg.twist.linear.y = 0.0
            self.last_dy_repeats = self.dy_repeats
        #Pauza 1 sekundu
        elif x_duration + pause_duration + y_duration + pause_duration + x_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration:
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = 0.0
        #Po Y osi
        elif x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration:
            self.desired_pose_twist_msg.pose.position.y = self.dy_repeats*dy_amplitude + vy_amplitude*(cycle_time - (x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration))
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = vy_amplitude
        #Pauza 1 sekundu
        elif x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration < cycle_time <= x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration + pause_duration:
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = 0.0
            if(self.dy_repeats == self.last_dy_repeats):
                self.dy_repeats += 1.0
    
        self.pose_twist_publisher.publish(self.desired_pose_twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SquareWaveTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
