from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("wcr_description"),
            "urdf",
            "mock_wcr_classic.urdf.xacro",
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("wcr_bringup"),
            "config",
            "wcr_controllers_swerve.yaml",
        ]
    )

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("wcr_description"),
            "rviz",
            "urdf_config.rviz",
        ]
    )

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # Global remaps apply to the controllers loaded in this process, so the
    # swerve controller takes /wcr/cmd_vel and publishes /wcr/odom like
    # inv_kin_controller + wcr_odometry.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/swerve_drive_controller/cmd_vel_unstamped", "/wcr/cmd_vel"),
            ("/swerve_drive_controller/odom", "/wcr/odom"),
            ("/swerve_drive_controller/reset_odometry", "/reset_odometry"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        remappings=[("/joint_states", "/wcr/joint_states")],
    )

    swerve_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Fixed frame odom so the robot model moves with the odometry.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path, '-f', 'odom'],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        swerve_drive_controller_spawner,
        rviz_node
    ])
