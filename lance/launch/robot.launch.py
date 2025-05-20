import os
import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_path = get_package_share_directory('lance')
    phx5_path = get_package_share_directory('phoenix5_driver')
    phx6_path = get_package_share_directory('phoenix6_driver')
    controller_path = get_package_share_directory('teleop_control')

    perception_live = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'live.launch.py') ),
        launch_arguments = {
            'foxglove' : 'false',
            'foxglove_mode' : 'live',
            'processing' : 'true',
            'record' : LaunchConfiguration('record_lidar', default='false'),
            'bag' : 'false',
            'disable_state_pub' : LaunchConfiguration('disable_state_pub', default='false') }.items(),
        condition = IfCondition( LaunchConfiguration('perception', default='true') )
    )

    phoenix5_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(phx5_path, 'launch', 'main.launch.py')
        ),
        condition = IfCondition(
            PythonExpression(["'true' if '", LaunchConfiguration('phoenix_driver', default='false'), "' == '5' else 'false'"]) )
    )
    phoenix6_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(phx6_path, 'launch', 'main.launch.py')
        ),
        condition = IfCondition(
            PythonExpression(["'true' if '", LaunchConfiguration('phoenix_driver', default='false'), "' == '6' else 'false'"]) )
    )

    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_path, 'launch', 'main.launch.py')
        ),
        condition = IfCondition( LaunchConfiguration('controller', default='true') )
    )

    # bag2 record
    motor_recorder = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'record',
            '-o', f"lance_motor_data_{ datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S") }",
            '/lance/robot_mode',
            '/lance/watchdog_feed',
            '/lance/track_left/ctrl',
            '/lance/track_left/faults',
            '/lance/track_left/info',
            '/lance/track_right/ctrl',
            '/lance/track_right/faults',
            '/lance/track_right/info',
            '/lance/trencher/ctrl',
            '/lance/trencher/faults',
            '/lance/trencher/info',
            '/lance/hopper_act/ctrl',
            '/lance/hopper_belt/ctrl',
            '/lance/hopper_belt/faults',
            '/lance/hopper_belt/info',
            '/rosout'
            # '--compression-mode', 'file',
            # '--compression-format', 'zstd'
        ],
        output='screen',
        condition = IfCondition( LaunchConfiguration('record_motor', default='false') )
    )

    # foxglove server if enabled
    foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments =
            {
                'use_sim_time': 'false',
                'mode' : LaunchConfiguration('foxglove_mode', default='live')
                # 'mode' : 'test'
            }.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='false'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('foxglove_mode', default_value='live'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('record_lidar', default_value='false'),
        DeclareLaunchArgument('record_motor', default_value='false'),
        DeclareLaunchArgument('disable_state_pub', default_value='false'),
        DeclareLaunchArgument('phoenix_driver', default_value='6'),
        DeclareLaunchArgument('controller', default_value='true'),
        perception_live,
        phoenix5_driver,
        phoenix6_driver,
        controller_node,
        motor_recorder,
        foxglove_bridge
    ])
