import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('lance')
    sim_pkg_path = get_package_share_directory('csm_sim')
    perception_pkg_path = get_package_share_directory('cardinal_perception')

    # launch robot_state_publisher using sim description
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items(),
        condition = IfCondition( LaunchConfiguration('use_state_pub', default='true') )
    )
    # perception stack
    perception_node = Node(
        name = 'cardinal_perception',
        package = 'cardinal_perception',
        executable = 'perception_node',
        output = 'screen',
        parameters = [
            os.path.join(perception_pkg_path, 'config', 'perception.yaml'),
            {
                'use_sim_time': True,
                'scan_topic': '/lance/lidar_scan',
                'imu_topic': '/lance/imu'
            }
        ],
        remappings = [
            ('filtered_scan', '/cardinal_perception/filtered_scan'),
            ('tags_detections', '/cardinal_perception/tags_detections'),
            ('map_cloud', '/cardinal_perception/map_cloud')
        ]
    )
    # foxglove server if enabled
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'true'}.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='true'))
    )

    isaac_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'isaac_sim.launch.py')
        ),
        launch_arguments = {
            'state_pub' : 'false',
            'foxglove' : 'false'
        }.items(),
        condition = IfCondition(
            AndSubstitution(
                LaunchConfiguration('isaac_sim'),
                NotSubstitution(
                    LaunchConfiguration('gz_sim')
                )
            )
        )
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments = {
            'state_pub' : 'false',
            'foxglove' : 'false'
        }.items(),
        condition = IfCondition(
            AndSubstitution(
                LaunchConfiguration('gz_sim'),
                NotSubstitution(
                    LaunchConfiguration('isaac_sim')
                )
            )
        )
    )
    tag_detection_node = Node(
        name = 'tags_detector',
        package = 'cardinal_perception',
        executable = 'tag_detection_node',
        output = 'screen',
        parameters = [
            os.path.join(perception_pkg_path, 'config', 'tag_detection.yaml'),
            { 'use_sim_time': True }
        ],
        remappings = [
            ('tags_detections', '/cardinal_perception/tags_detections')
        ],
        condition = IfCondition(
            AndSubstitution(
                LaunchConfiguration('gz_sim'),
                NotSubstitution(
                    LaunchConfiguration('isaac_sim')
                )
            )
        )
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_state_pub', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('isaac_sim', default_value='false'),
        DeclareLaunchArgument('gz_sim', default_value='false'),
        robot_state_publisher,
        perception_node,
        foxglove_node,
        isaac_sim,
        gz_sim,
        tag_detection_node
    ])
