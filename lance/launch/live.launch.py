import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('lance')
    sim_pkg_path = get_package_share_directory('csm_sim')

    # launch robot_state_publisher using sim description
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'false'}.items(),
        # condition = IfCondition( PythonExpression(["'true' if '", LaunchConfiguration('bag', default='false'), "' == 'false' else 'false'"]) )
    )
    # lidar driver
    multiscan_driver = Node(
        name = 'multiscan_driver',
        package = 'multiscan_driver',
        executable = 'multiscan_driver',
        output = 'screen',
        parameters = [
            os.path.join(pkg_path, 'config', 'multiscan_driver.yaml')
        ],
        remappings = [
            ('lidar_scan', '/multiscan/lidar_scan'),
            ('lidar_imu', '/multiscan/imu')
        ],
        condition = IfCondition( PythonExpression(["'true' if '", LaunchConfiguration('bag', default='false'), "' == 'false' else 'false'"]) )
    )
    # perception stack
    launch_perception = Node(
        name = 'cardinal_perception',
        package = 'cardinal_perception',
        executable = 'perception_node',
        output = 'screen',
        parameters = [
            os.path.join(pkg_path, 'config', 'perception_live.yaml'),
            {
                'use_sim_time': False,
                'scan_topic': '/multiscan/lidar_scan',
                'imu_topic': '/multiscan/imu'
            }
        ],
        remappings = [
            ('filtered_scan', '/cardinal_perception/filtered_scan'),
            ('tags_detections', '/cardinal_perception/tags_detections'),
            ('map_cloud', '/cardinal_perception/map_cloud')
        ],
        condition = IfCondition( LaunchConfiguration('processing', default='true') ),
        # prefix=['xterm -e gdb -ex run --args']
        # prefix=['valgrind --leak-check=yes -v']
        # prefix=['valgrind --tool=callgrind --dump-instr=yes --simulate-cache=yes --collect-jumps=yes']
    )
    # bag2 record
    bag_recorder = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'record',
            # '-o', '<OUTPUT FILE HERE>',
            '/multiscan/lidar_scan',
            '/multiscan/imu',
            # '/cardinal_perception/tags_detections',
            '/tf',
            '/tf_static',
            # '--compression-mode', 'file',
            # '--compression-format', 'zstd'
        ],
        output='screen',
        condition = IfCondition( LaunchConfiguration('record', default='false') )
    )
    # bag2 play
    bag_player = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag', default=''),
            # '--loop',
            # '--rate', '0.5',
            '--topics', '/multiscan/lidar_scan', '/multiscan/imu', '/cardinal_perception/tags_detections',
            # '--start-offset', '248'
            '--start-paused'
        ],
        output='screen',
        condition = IfCondition( PythonExpression(["'false' if '", LaunchConfiguration('bag', default='false'), "' == 'false' else 'true'"]) )
    )
    # foxglove server if enabled
    foxglove_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments =
            {
                'use_sim_time': 'false',
                'mode' : LaunchConfiguration('mode', default='live')
            }.items(),
        condition = IfCondition(LaunchConfiguration('foxglove', default='true'))
    )


    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_mode', default_value='live'),
        DeclareLaunchArgument('processing', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('bag', default_value='false'),
        robot_state_publisher,
        multiscan_driver,
        launch_perception,
        bag_recorder,
        bag_player,
        foxglove_node
    ])
