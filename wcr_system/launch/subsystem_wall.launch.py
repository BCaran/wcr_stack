import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
	vesc_config = os.path.join(get_package_share_directory('wcr_system'), 'config', 'vesc_config.yaml')
	return LaunchDescription([
		Node(
           	package='teensy_serial',
            executable='teensy_serial_node',
        ),
        Node(
			package = 'vesc_driver',
			executable = 'vesc_driver_node',
			parameters = [vesc_config]
		),

    ])
