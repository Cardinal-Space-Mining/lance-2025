from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    pkg_path = get_package_share_directory('camera_utils')
    subdir = LaunchConfiguration('config_subdir')

    camera_node = Node(
        name = 'camera_driver',
        package = 'usb_cam',
        executable = 'usb_cam_node_exe',
        output = 'screen',
        namespace = subdir,
        parameters = [
            PathJoinSubstitution([pkg_path, 'config', subdir, 'params.yaml']),
            { 'camera_info_url': PathJoinSubstitution(['file://' + pkg_path, 'config', subdir, 'calibration.yaml']) }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_subdir'),
        camera_node
    ])
