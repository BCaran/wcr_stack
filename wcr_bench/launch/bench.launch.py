"""Launch the bench force node with its YAML configuration.

Usage:
    ros2 launch wcr_bench bench.launch.py
    ros2 launch wcr_bench bench.launch.py port:=/dev/bench_nano

The port argument overrides whatever the YAML says, which is handy when
moving between machines without editing the file. Leaving it out uses
the YAML value.

Implementation note: the override goes through OpaqueFunction rather
than a plain parameter dict. A LaunchConfiguration is only resolved at
launch time, so passing it unconditionally pushes an empty string over
the YAML value whenever the argument is omitted - which blanks the port
instead of falling back to it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_node(context, *args, **kwargs):
    params_file = os.path.join(
        get_package_share_directory('wcr_bench'),
        'config',
        'bench_params.yaml',
    )

    parameters = [params_file]

    port = LaunchConfiguration('port').perform(context)
    if port:
        parameters.append({'port': port})

    return [
        Node(
            package='wcr_bench',
            executable='bench_forces',
            name='bench_forces',
            output='screen',
            parameters=parameters,
            emulate_tty=True,
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='',
            description='serial port override, e.g. /dev/bench_nano; '
                        'leave empty to use the value in the YAML',
        ),
        OpaqueFunction(function=make_node),
    ])
