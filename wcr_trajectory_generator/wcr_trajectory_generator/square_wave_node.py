import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from scipy import signal
import numpy as np
from wcr_interfaces.msg import DesiredPoseTwist
from scipy.interpolate import interp1d

x_amplitude = 0.3
dy_amplitude = -0.1
vx_amplitude = 0.05
vy_amplitude = -0.05

# Parameters for the custom wave pattern (x-t)
x_duration = x_amplitude/vx_amplitude  # Duration of positive phase for x-t (seconds)
y_duration = dy_amplitude/vy_amplitude
pause_duration = 2  # Duration of pause for both pauses in x-t (seconds)
repeats = 3  # Number of repetitions
sampling_rate = 1000  # Samples per second

# Total duration of one cycle (positive, first pause, negative, second pause)
cycle_duration = x_duration + pause_duration + y_duration + pause_duration + x_duration + pause_duration + y_duration + pause_duration
total_duration = cycle_duration * repeats

class SquareWaveTrajectoryNode(Node):
    def __init__(self):
        super().__init__('square_wave_trajectory')

        # Publishers
        self.pose_twist_publisher = self.create_publisher(DesiredPoseTwist, '/wcr/desired_pose_twist', 10)
        self.path_publisher = self.create_publisher(Path, '/wcr/desired_path', 10)
        self.filled_poses_ = False
        self.path_msg_ = Path()
        self.path_pose_ = PoseStamped()
        self.calculate_full_path()
        
        self.timer = self.create_timer(1/sampling_rate, self.trajectory_callback)
        #self.timer = self.create_timer(1.0, self.path_calblback)

        self.start_time = self.get_clock().now().nanoseconds
        self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9

        self.dy_repeats = 0.0
        self.last_dy_repeats = 0.0

        self.desired_pose_twist_msg = DesiredPoseTwist()

    def trajectory_callback(self):
        if self.dy_repeats < repeats:
            self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
            self.desired_pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.desired_pose_twist_msg.header.frame_id = "odom"
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
                self.desired_pose_twist_msg.twist.linear.y = vy_amplitude * 1e-3
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
                self.desired_pose_twist_msg.twist.linear.y = vy_amplitude * 1e-3
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
        else:
            self.desired_pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.desired_pose_twist_msg.header.frame_id = "base_link"
            self.pose_twist_publisher.publish(self.desired_pose_twist_msg)
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = 0.0

    def path_calblback(self):
        self.path_msg_.header.stamp = self.get_clock().now().to_msg()
        self.path_msg_.header.frame_id = "odom"

        self.path_pose_.header = self.path_msg_.header
        self.path_publisher.publish(self.path_msg_)


    def calculate_full_path(self):
        path_time = np.linspace(0, total_duration, int(total_duration * sampling_rate))
        path_pause = pause_duration + y_duration
        dotx_t = np.zeros_like(path_time)
        for i, t in enumerate(path_time):
            cycle_time = t % cycle_duration  # Time within the current cycle
            if 0 <= cycle_time < x_duration:
                dotx_t[i] = vx_amplitude  # Positive phase
            elif x_duration <= cycle_time < x_duration + path_pause:
                dotx_t[i] = 0  # Pause phase after positive
            elif x_duration + path_pause <= cycle_time < x_duration + path_pause + x_duration:
                dotx_t[i] = -vx_amplitude  # Negative phase
            elif x_duration + path_pause + x_duration <= cycle_time < cycle_duration:
                dotx_t[i] = 0  # Pause phase after negative
        self.x_t = np.cumsum(dotx_t) * (path_time[1] - path_time[0])
        positive_duration_y = 41  # Duration of positive phase for y-t (seconds)
        negative_duration_y = 6  # Duration of negative phase for y-t (seconds)

        # Total duration of one cycle for y-t
        cycle_duration_y = positive_duration_y + negative_duration_y
        total_duration_y = cycle_duration_y * repeats

        # Generate time vector for y-t
        time_y = np.linspace(0, total_duration_y, int(total_duration_y * sampling_rate))

        # Create dot_y_t pattern
        dot_y_t = np.zeros_like(time_y)
        for i, t in enumerate(time_y):
            cycle_time = t % cycle_duration_y  # Time within the current cycle
            if 0 <= cycle_time < positive_duration_y:
                dot_y_t[i] = 0  # Positive phase (always 0)
            elif positive_duration_y <= cycle_time < positive_duration_y + negative_duration_y:
                dot_y_t[i] = -vx_amplitude  # Negative phase for y-t

        # Compute y-t as the integral of dot_y-t
        y_t = np.cumsum(dot_y_t) * (time_y[1] - time_y[0])
        # Interpolate y_t onto the time grid of x_t
        interp_y_t = interp1d(time_y, y_t, kind='linear', fill_value="extrapolate")
        self.y_t_interpolated = interp_y_t(path_time)

        for x, y in zip(self.x_t, self.y_t_interpolated):
            self.path_pose_.pose.position.x = x
            self.path_pose_.pose.position.y = y
            self.path_pose_.pose.position.z = 0.0
            self.path_pose_.pose.orientation.x = 0.0
            self.path_pose_.pose.orientation.y = 0.0
            self.path_pose_.pose.orientation.z = 0.0
            self.path_pose_.pose.orientation.w = 1.0

            self.path_msg_.poses.append(self.path_pose_)

        

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
