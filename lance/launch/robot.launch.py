import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
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
            'foxglove' : LaunchConfiguration('foxglove', default='false'),
            'foxglove_mode' : LaunchConfiguration('foxglove_mode', default='live'),
            'processing' : 'true',
            'record' : LaunchConfiguration('record', default='false'),
            'bag' : 'false' }.items(),
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

    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('foxglove_mode', default_value='live'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('phoenix_driver', default_value='6'),
        DeclareLaunchArgument('controller', default_value='true'),
        perception_live,
        phoenix5_driver,
        phoenix6_driver,
        controller_node
    ])
