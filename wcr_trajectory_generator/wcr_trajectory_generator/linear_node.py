import rclpy
from rclpy.node import Node
from wcr_interfaces.msg import DesiredPoseTwist

line_angle = 0.0
line_length = 1.0
vx_amplitude = 0.05
vy_amplitude = 0.05
sampling_rate = 1000  # Samples per second

class LinearTrajectoryNode(Node):

    def __init__(self):
        super().__init__('linear_trajectory')
        self.publisher_ = self.create_publisher(DesiredPoseTwist, 'topic', 10)
        self.timer = self.create_timer(1/sampling_rate, self.trajectory_callback)
        self.start_time = self.get_clock().now().nanoseconds
        self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
        self.desired_pose_twist_msg = DesiredPoseTwist()

    def trajectory_callback(self):
        self.current_time_s = (self.get_clock().now().nanoseconds - self.start_time) * 1e-9
        self.desired_pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.desired_pose_twist_msg.header.frame_id = "odom"

        self.desired_pose_twist_msg.po


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