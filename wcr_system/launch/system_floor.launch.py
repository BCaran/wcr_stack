import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
	
	realsense_t265_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(['/home/wcr/wcr_ws/src/realsense-ros-3.2.3/realsense2_camera/launch/rs_t265_launch.py']))
	ps4_joystick_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(['/home/wcr/wcr_ws/src/wcr_system/launch/ps4.launch.py']))
	marvelmind_config = os.path.join(get_package_share_directory('wcr_system'), 'config', 'marvelmind_ros2_config.yaml')
	robot_description_config = xacro.process_file('/home/wcr/wcr_ws/src/wcr_description/urdf/wcr.urdf.xacro')
	fws_fwd_config = '/home/wcr/wcr_ws/src/wcr_system/config/fws_fwd_config.yaml'
	bno055_config = '/home/wcr/wcr_ws/src/wcr_system/config/bno055_params_i2c.yaml'
	twist_mux_params = os.path.join(get_package_share_directory('wcr_system'),'config', 'wcr_twist_mux_topics.yaml')

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

		#Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments = ['0', '0', '0', '0', '0', '0', 'world', 'odom']
        #),

		#Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments = ['0.06579', '0', '0.08934', '0', '0', '0', 'base_link', 'imu_link']
        #),

		Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml(), "publish_frequency": 50.0}],
            remappings=[('/joint_states', '/wcr/joint_states')]
            ),

		Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', '/wcr/cmd_vel')},
            parameters=[twist_mux_params]
        ),

		realsense_t265_launch,
		ps4_joystick_launch
		#Node(
		#	package = 'marvelmind_ros2',
		#	executable = 'marvelmind_ros2',
		#	parameters = [marvelmind_config]
		#),
		#Node(
		#	package = 'wcr_system',
		#	executable = 'marvelmind_pose_converter'
		#),
		#realsense_launch
		#0.06579 0 0.08934
	])
