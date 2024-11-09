import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    tag_detector = Node(
        name = 'cardinal_perception_tag_detection',
        package = 'cardinal_perception',
        executable = 'tag_detection_node',
        output = 'screen',
        parameters = [
            os.path.join(get_package_share_directory('cardinal_perception'), 'config', 'tag_detection.yaml'),
            { 'use_sim_time': True }
        ],
        remappings = [ ('tags_detections', '/cardinal_perception/tags_detections') ]
    )
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tags_server'), 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='false'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='false'),
        tag_detector,
        foxglove_node
    ])
