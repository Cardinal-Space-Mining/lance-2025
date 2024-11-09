import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_path = get_package_share_directory('camera_utils')
    itf_use_ip = LaunchConfiguration('use_ip_driver', default='false')
    subdir = LaunchConfiguration('config_subdir')

    usb_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'run_usb.launch.py')),
        launch_arguments = {
            'config_subdir': subdir
        }.items(),
        condition = UnlessCondition(itf_use_ip)
    )
    ip_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'run_ip.launch.py')),
        launch_arguments = {
            'config_subdir': subdir
        }.items(),
        condition = IfCondition(itf_use_ip)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_ip_driver', default_value = 'false'),
        DeclareLaunchArgument('config_subdir'),
        usb_driver,
        ip_driver
    ])