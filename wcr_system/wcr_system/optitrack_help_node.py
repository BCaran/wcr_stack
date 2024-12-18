import rclpy
from rclpy.node import Node

import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Transform
from std_srvs.srv import Trigger



class OptitrackHelp(Node):
    def __init__(self):
        super().__init__('optitrack_help_node')
        self.optitrack_pose_sub = self.create_subscription(PoseStamped, '/optitrack/wcr/pose', self.optitrack_pose_callback, 10)
        self.optitrack_pose_sub  # prevent unused variable warning
        self.tranformed_pose_pub = self.create_publisher(PoseStamped, 'optitrack/pose', 10)
        self.optitrack_transform_stamped = self.create_publisher(TransformStamped, 'optitrack/wcr/transform', 10)
        self.optitrack_reset_srv = self.create_service(Trigger, "reset_optitrack", self.reset_optitrack_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sending_transform_ = TransformStamped()
        self.reset_transform_ = TransformStamped()
        self.transform = TransformStamped()
        self.transformed_pose = PoseStamped()
        self.reset_transform_.transform.translation.x = 0.0
        self.reset_transform_.transform.translation.y = 0.0
        self.reset_transform_.transform.translation.z = 0.0
        self.reset_transform_.transform.rotation.x = 0.0
        self.reset_transform_.transform.rotation.y = 0.0
        self.reset_transform_.transform.rotation.z = 0.0
        self.reset_transform_.transform.rotation.w = 1.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.z_ = 0.0
        self.rx_ = 0.0
        self.ry_ = 0.0
        self.rz_ = 0.0
        self.rw_ = 0.0

    def reset_optitrack_callback(self, request, response):
        self.reset_transform_.transform.translation.x = self.x_
        self.reset_transform_.transform.translation.y = self.y_
        self.reset_transform_.transform.translation.z = self.z_
        self.reset_transform_.transform.rotation.x = self.rx_
        self.reset_transform_.transform.rotation.y = self.ry_
        self.reset_transform_.transform.rotation.z = self.rz_
        self.reset_transform_.transform.rotation.w = self.rw_
        response.message = "Optitrack reseted"
        response.success = True
        return response
    
    def transform_to_matrix(self, transform):
        translation = (transform.transform.translation.x,
                       transform.transform.translation.y,
                       transform.transform.translation.z)
        rotation = (transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w)
        return tf_transformations.translation_matrix(translation) @ tf_transformations.quaternion_matrix(rotation)

    def matrix_to_transform(self, matrix):
        translation = tf_transformations.translation_from_matrix(matrix)
        rotation = tf_transformations.quaternion_from_matrix(matrix)
        transform = TransformStamped()
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        return transform

    def optitrack_pose_callback(self, msg):
        #Matrix multiplication for reseting
        self.transform.transform.translation.x = msg.pose.position.x
        self.transform.transform.translation.y = msg.pose.position.y
        self.transform.transform.translation.z = msg.pose.position.z
        self.transform.transform.rotation.x = msg.pose.orientation.x
        self.transform.transform.rotation.y = msg.pose.orientation.y
        self.transform.transform.rotation.z = msg.pose.orientation.z
        self.transform.transform.rotation.w = msg.pose.orientation.w

        matrix1 = self.transform_to_matrix(self.transform)
        matrix2 = self.transform_to_matrix(self.reset_transform_)

        # Multiply the matrices
        result_matrix = tf_transformations.concatenate_matrices(tf_transformations.inverse_matrix(matrix2), matrix1)

        self.sending_transform_ = self.matrix_to_transform(result_matrix)

        self.x_ = self.sending_transform_.transform.translation.x
        self.y_ = self.sending_transform_.transform.translation.y
        self.z_ = 0.0
        self.rx_ = self.sending_transform_.transform.rotation.x
        self.ry_ = self.sending_transform_.transform.rotation.y
        self.rz_ = self.sending_transform_.transform.rotation.z
        self.rw_ = self.sending_transform_.transform.rotation.w

        self.sending_transform_.header.stamp = msg.header.stamp
        self.sending_transform_.header.frame_id = msg.header.frame_id
        self.sending_transform_.child_frame_id = "optitrack_wcr_link"

        self.transformed_pose.header.frame_id = self.sending_transform_.header.frame_id
        self.transformed_pose.header.stamp = self.sending_transform_.header.stamp
        self.transformed_pose.pose.position.x = self.sending_transform_.transform.translation.x
        self.transformed_pose.pose.position.y = self.sending_transform_.transform.translation.y
        self.transformed_pose.pose.position.z = 0.0
        self.transformed_pose.pose.orientation.x = self.sending_transform_.transform.rotation.x
        self.transformed_pose.pose.orientation.y = self.sending_transform_.transform.rotation.y
        self.transformed_pose.pose.orientation.z = self.sending_transform_.transform.rotation.z
        self.transformed_pose.pose.orientation.w = self.sending_transform_.transform.rotation.w
        self.tranformed_pose_pub.publish(self.transformed_pose)
        self.optitrack_transform_stamped.publish(self.sending_transform_)
        self.tf_broadcaster.sendTransform(self.sending_transform_)

        

def main(args=None):
    rclpy.init(args=args)

    optitrack_help = OptitrackHelp()

    rclpy.spin(optitrack_help)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    optitrack_help.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()