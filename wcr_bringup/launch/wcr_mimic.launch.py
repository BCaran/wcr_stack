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
            "wcr_mimic.urdf.xacro",
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("wcr_bringup"),
            "config",
            "wcr_controllers_mimic.yaml",
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

    main_driving_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["main_driving_controller", "--controller-manager", "/controller_manager"],
    )

    main_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["main_steering_controller", "--controller-manager", "/controller_manager"],
    )
    
    mimic_driving_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mimic_driving_controller", "--controller-manager", "/controller_manager"],
    )
    
    mimic_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mimic_steering_controller", "--controller-manager", "/controller_manager"],
    )
    
    mimic_controller_help_node = Node(
        package="wcr_controllers",
        executable="mimic_help_controller",
    )
    
    odometry_node = Node(
        package="wcr_odometry",
        executable="odometry",
    )
    

    return LaunchDescription([
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        main_driving_controller_spawner,
        main_steering_controller_spawner,
        mimic_driving_controller_spawner,
        mimic_steering_controller_spawner,
        mimic_controller_help_node,
        odometry_node
    ])