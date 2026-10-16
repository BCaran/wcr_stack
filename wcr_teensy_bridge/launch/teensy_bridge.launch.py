from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("wcr_teensy_bridge"))
    parameters_file = package_share / "config" / "teensy_bridge.yaml"

    return LaunchDescription(
        [
            Node(
                package="wcr_teensy_bridge",
                executable="teensy_bridge_node",
                name="teensy_bridge",
                output="screen",
                parameters=[str(parameters_file)],
            )
        ]
    )
