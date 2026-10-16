from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# (steering_mode, use_mock_hardware) -> (urdf, controller config)
SETUPS = {
    ("position", "false"): ("wcr_classic.urdf.xacro", "wcr_controllers_swerve.yaml"),
    ("position", "true"): ("mock_wcr_classic.urdf.xacro", "wcr_controllers_swerve.yaml"),
    ("velocity", "false"): ("wcr_velocity.urdf.xacro", "wcr_controllers_swerve_velocity.yaml"),
}


def launch_setup(context):
    steering_mode = LaunchConfiguration("steering_mode").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)
    key = (steering_mode, use_mock_hardware)
    if key not in SETUPS:
        raise RuntimeError(
            f"Unsupported combination steering_mode:={steering_mode} "
            f"use_mock_hardware:={use_mock_hardware}. Supported: {sorted(SETUPS)}"
        )
    urdf_file, controllers_file = SETUPS[key]

    urdf_path = PathJoinSubstitution([FindPackageShare("wcr_description"), "urdf", urdf_file])
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("wcr_bringup"), "config", controllers_file]
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Global remaps apply to the controllers loaded in this process, so the
    # swerve controller replaces inv_kin_controller + wcr_odometry on the
    # existing topics.
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

    return [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        swerve_drive_controller_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "steering_mode",
            default_value="position",
            description="Steering command interface: position or velocity.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock_components instead of the Dynamixel hardware "
                        "(position steering only).",
        ),
        OpaqueFunction(function=launch_setup),
    ])
