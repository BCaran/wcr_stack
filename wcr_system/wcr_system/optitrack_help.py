import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import String


class OptitrackTransformer(Node):

    def __init__(self):
        super().__init__('optitrack_help_node')
        self.subscription = self.create_subscription(String,'topic',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()