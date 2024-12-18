import rclpy
from rclpy.node import Node
from marvelmind_ros2_msgs.msg import HedgePosition
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger


class MarvelmindPoseConverter(Node):

    def __init__(self):
        super().__init__('marvelmind_pose_converter')
        self.subscription = self.create_subscription(HedgePosition, 'hedgehog_pos', self.marvelmind_callback, 10)
        self.subscription  # prevent unused variable warning
        self.pose_pub = self.create_publisher(PoseStamped, "marvelmind/pose", 10)
        self.pose_msg_ = PoseStamped()
        #self.optitrack_reset_srv = self.create_service(Trigger, "reset_marvelmind", self.reset_marvelmind_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sending_transform_ = TransformStamped()

    def marvelmind_callback(self, msg):
        self.pose_msg_.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg_.header.frame_id = "world"
        self.pose_msg_.pose.position.x = msg.x_m
        self.pose_msg_.pose.position.y = msg.y_m
        self.pose_msg_.pose.position.z = msg.z_m
        self.sending_transform_.header.frame_id = "world"
        self.sending_transform_.child_frame_id = "marvelmind_link"
        self.sending_transform_.header.stamp = self.get_clock().now().to_msg()
        self.sending_transform_.transform.translation.x = msg.x_m
        self.sending_transform_.transform.translation.y = msg.y_m
        self.sending_transform_.transform.translation.z = msg.z_m
        self.tf_broadcaster.sendTransform(self.sending_transform_)
        self.pose_pub.publish(self.pose_msg_)



def main(args=None):
    rclpy.init(args=args)

    marvelmind_pose_converter_node = MarvelmindPoseConverter()

    rclpy.spin(marvelmind_pose_converter_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    marvelmind_pose_converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()