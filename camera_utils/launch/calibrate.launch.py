import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_path = get_package_share_directory('camera_utils')
    subdir = LaunchConfiguration('config_subdir')

    camera_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'camera_driver.launch.py')),
        launch_arguments = {
            'use_ip_driver': LaunchConfiguration('use_ip_driver', default='false'),
            'config_subdir': subdir
        }.items()
    )
    calibrate_node = Node(
        name = 'camera_calibrator',
        package = 'camera_calibration',
        executable = 'cameracalibrator',
        output = 'screen',
        remappings = [
            ('image', PathJoinSubstitution([subdir, 'image_raw'])),
            ('camera', PathJoinSubstitution([subdir, 'camera_info']))
        ],
        arguments = [
            '--pattern=chessboard',
            '--size=9x6',
            '--square=0.1778',
            '--k-coefficients=2',
            '--no-service-check'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_ip_driver', default_value = 'false'),
        DeclareLaunchArgument('config_subdir'),
        camera_driver,
        calibrate_node
    ])
