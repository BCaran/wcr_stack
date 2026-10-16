from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("wcr_description"),
            "urdf",
            "wcr_effort.urdf.xacro",
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("wcr_bringup"),
            "config",
            "wcr_controllers_chained.yaml",
        ]
    )

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        remappings=[("/joint_states", "/wcr/joint_states")],
    )

    driving_pid_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pid_controller_fl_wheel", "--controller-manager", "/controller_manager"],
    )
    
    odometry_node = Node(
        package="wcr_odometry",
        executable="odometry",
    )
    

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        driving_pid_spawner,
        odometry_node
    ])