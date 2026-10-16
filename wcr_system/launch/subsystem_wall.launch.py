import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    vesc_config = os.path.join(
        get_package_share_directory('wcr_system'),
        'config',
        'vesc_config.yaml'
    )
    
    teensy_config = os.path.join(
            get_package_share_directory('wcr_teensy_bridge'),
            'config',
            'teensy_bridge.yaml'
        )
    
    teensy_node = Node(
        package="wcr_teensy_bridge",
        executable="teensy_bridge_node",
        name="teensy_bridge",
        output="screen",
        parameters=[str(teensy_config)],
    )

    vesc_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[vesc_config],
        remappings=[
            (
                '/commands/motor/duty_cycle',
                '/wcr/edf/duty_cycle',
            ),
        ],

        # Restart the driver if the initial UART connection fails.
        respawn=True,
        respawn_delay=2.0,
    )

    delayed_vesc_node = TimerAction(
        period=3.0,
        actions=[vesc_node],
    )

    return LaunchDescription([
        teensy_node,
        delayed_vesc_node,
    ])