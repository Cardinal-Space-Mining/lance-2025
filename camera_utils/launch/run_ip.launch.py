import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('camera_utils')
    subdir = LaunchConfiguration('config_subdir')

    ipcamera_node = ComposableNode(
        package = 'ros2_ipcamera',
        plugin = 'ros2_ipcamera::IpCamera',
        name = 'ipcamera',
        parameters = [
            PathJoinSubstitution([pkg_path, 'config', subdir, 'params.yaml']),
            { 'camera_calibration_file': PathJoinSubstitution(['file://' + pkg_path, 'config', subdir, 'calibration.yaml']) }
        ]
    )
    container = ComposableNodeContainer(
        name = 'container',
        namespace = subdir,
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [ipcamera_node],
        output = 'screen',
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument('config_subdir'),
        container
    ])
