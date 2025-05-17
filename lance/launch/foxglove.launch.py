import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_conditional_node_config(fname, condition):
    return Node(
            name = 'foxglove_server',
            package = 'foxglove_bridge',
            executable = 'foxglove_bridge',
            output = 'screen',
            parameters = [
                fname,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
                }
            ],
            condition = condition )

def generate_launch_description():

    pkg_path = get_package_share_directory('lance')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mode', default_value='live'),
        generate_conditional_node_config(
            os.path.join(pkg_path, 'config', 'foxglove_bridge_live.yaml'),
            IfCondition( PythonExpression(["'false' if '", LaunchConfiguration('mode', default='live'), "' == 'test' else 'true'"]) ) ),
        generate_conditional_node_config(
            os.path.join(pkg_path, 'config', 'foxglove_bridge_test.yaml'),
            IfCondition( PythonExpression(["'true' if '", LaunchConfiguration('mode', default='live'), "' == 'test' else 'false'"]) ) ),
    ])
