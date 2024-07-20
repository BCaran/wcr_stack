import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	bno055_config = os.path.join(get_package_share_directory('wcr_system'), 'config', 'bno055_params_i2c.yaml')
	marvelmind_config = os.path.join(get_package_share_directory('wcr_system'), 'config', 'marvelmind_ros2_config.yaml')
	realsense_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('wcr_system'), 'launch'),'/rs_custom.launch.py']))
	fws_fwd_config = os.path.join(get_package_share_directory('wcr_system'), 'config', 'fws_fwd_config.yaml')
	return LaunchDescription([
		Node(
			package='fws_fwd_dxl',
			executable='fwsfwd_controller',
			parameters = [fws_fwd_config]
		),
		Node(
			package = 'bno055',
			executable = 'bno055',
			parameters = [bno055_config]
		),
		Node(
			package = 'marvelmind_ros2',
			executable = 'marvelmind_ros2',
			parameters = [marvelmind_config]
		),
		realsense_launch
	])
