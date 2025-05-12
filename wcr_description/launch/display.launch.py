import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    robot_name = "wcr"
    package_name = "wcr_description"
    rviz_config = os.path.join(get_package_share_directory(
        package_name), "rviz", "wcr.rviz")
    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", "wcr.urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml(), "publish_frequency": 50.0}],
            remappings=[('/joint_states', '/wcr/joint_states')]
            ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen")
    ])
