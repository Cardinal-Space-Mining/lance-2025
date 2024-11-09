import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('tags_server')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            name = 'foxglove_server',
            package = 'foxglove_bridge',
            executable = 'foxglove_bridge',
            output = 'screen',
            parameters = [
                os.path.join(pkg_path, 'config', 'foxglove_bridge.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}
            ]
        )
    ])
