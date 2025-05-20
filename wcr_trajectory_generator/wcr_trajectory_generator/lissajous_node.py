import rclpy
from rclpy.node import Node
from wcr_interfaces.msg import DesiredPoseTwist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf_transformations import quaternion_from_euler

v_max = 0.05
duration = 2*np.pi/v_max
a = 1.0 #horizontal amplitude
b = 0.5 #vertical amplitude
omega = 0.05#base angular velocity
phase = 0.0 #phase offset for x
sampling_rate = 1000
angle_follow_path = False


class CircularTrajectoryNode(Node):

    def __init__(self):
        super().__init__('lissajous_trajectory_node')
        self.publisher_ = self.create_publisher(DesiredPoseTwist, '/wcr/desired_pose_twist', 10)
        self.path_publisher = self.create_publisher(Path, '/wcr/desired_path', 10)
        
        self.timer = self.create_timer(1/sampling_rate, self.trajectory_callback)
        self.calculate_path()
        self.path_timer = self.create_timer(1.0, self.publish_total_path)
        self.desired_pose_twist_msg = DesiredPoseTwist()
        
        self.get_logger().info("a: %f" % a)
        self.get_logger().info("b: %f" % b)
        self.get_logger().info("Omega: %f" % omega)
        self.get_logger().info("Phase: %f" % phase)
        self.get_logger().info("Duration: %f" % duration)
        if angle_follow_path == True:
            self.get_logger().info("Theta: atan2(vy, vx)")
        else:
            self.get_logger().info("Theta: 0 rad")
            
        self.start_time = self.get_clock().now().nanoseconds
        self.current_time = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
        
        
                
    def calculate_path(self):
        #izračun putanje
        dt = 2
        t = np.arange(0, duration+dt, dt)
        x_d = a * np.sin(omega * t + phase)
        y_d = b * np.sin(2 * omega * t)
        v_x = a * omega * np.cos(omega * t + phase)
        v_y = 2 * b * omega * np.cos(2 * omega * t)
            
        if angle_follow_path == True:
            theta_d = np.unwrap(np.arctan2(v_y, v_x))
        else:
            theta_d = np.zeros(len(t))
        
        v_x_max = np.max(v_x)
        v_y_max = np.max(v_y)
        if v_x_max > 0.1:
            self.get_logger().warn("Max velocity v_x: %f" % v_x_max)
        else:
            self.get_logger().info("Max velocity v_x: %f" % v_x_max)
            
        if v_y_max > 0.1:
            self.get_logger().warn("Max velocity v_y: %f" % v_y_max)
        else:
            self.get_logger().info("Max velocity v_y: %f" % v_y_max)
        
        #spremanje putanje
        self.path = Path()
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = 'odom'  # or whatever fixed frame you use
        for x, y, theta in zip(x_d, y_d, theta_d):
            pose = PoseStamped()
            
            pose.header.stamp = self.path.header.stamp
            pose.header.frame_id = self.path.header.frame_id
            
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            q = quaternion_from_euler(0.0, 0.0, float(theta))
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            self.path.poses.append(pose)
        
    def publish_total_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)

    def trajectory_callback(self):
        if self.current_time <= duration:
            self.current_time = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
            t = self.current_time
            self.desired_pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.desired_pose_twist_msg.header.frame_id = "odom"
            
            self.desired_pose_twist_msg.pose.position.x = a * np.sin(omega * t + phase)
            self.desired_pose_twist_msg.pose.position.y = b * np.sin(2 * omega * t)
            self.desired_pose_twist_msg.twist.linear.x = a * omega * np.cos(omega * t + phase)
            self.desired_pose_twist_msg.twist.linear.y = 2 * b * omega * np.cos(2 * omega * t)
        else:
            self.desired_pose_twist_msg.twist.linear.x = 0.0
            self.desired_pose_twist_msg.twist.linear.y = 0.0
            self.publisher_.publish(self.desired_pose_twist_msg) 
            self.get_logger().info("Trajectory generator finished")
            self.destroy_node()
            rclpy.shutdown()
            
        self.publisher_.publish(self.desired_pose_twist_msg)         
            
def main(args=None):
    rclpy.init(args=args)
    node = CircularTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()