import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # launch robot_state_publisher using sim description
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('csm_gz_sim'), 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items(),
        condition = IfCondition( LaunchConfiguration('use_state_pub', default='true') )
    )
    # perception stack
    launch_localization = Node(
        name = 'cardinal_perception_localization',
        package = 'cardinal_perception',
        executable = 'localization_node',
        output = 'screen',
        parameters = [
            os.path.join(get_package_share_directory('cardinal_perception'), 'config', 'localization.yaml'),
            {
                'use_sim_time': True,
                # 'require_rebias_before_tf_pub': False,
                # 'require_rebias_before_scan_pub': False
            }
        ],
        remappings = [
            ('filtered_scan', '/cardinal_perception/filtered_scan'),
            ('tags_detections', '/cardinal_perception/tags_detections')
        ]
    )
    # foxglove server if enabled
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lance'), 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'false'}.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='false'))
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_state_pub', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        robot_state_publisher,
        launch_localization,
        foxglove_node
    ])
