import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, OrSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('lance')
    sim_pkg_path = get_package_share_directory('csm_sim')
    phx5_path = get_package_share_directory('phoenix5_driver')
    phx6_path = get_package_share_directory('phoenix6_driver')
    controller_path = get_package_share_directory('teleop_control')

    is_state_pub_disabled = LaunchConfiguration('disable_state_pub', default='false')
    is_running_perception = LaunchConfiguration('perception', default='true')
    is_logging_lidar = LaunchConfiguration('record_lidar', default='false')
    is_logging_motor = LaunchConfiguration('record_motor', default='false')
    is_lidar_driver_forced = LaunchConfiguration('force_lidar_driver', default='false')
    phoenix_driver_version = LaunchConfiguration('phoenix_driver', default='false')
    arduino_device_id = LaunchConfiguration('arduino_device', default='/dev/ttyACM0')
    is_running_controller = LaunchConfiguration('controller', default='true')
    is_running_foxglove_bridge = LaunchConfiguration('foxglove', default='false')
    foxglove_config_mode = LaunchConfiguration('foxglove_mode', default='live')


    # launch robot_state_publisher using sim description
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments = {'use_sim_time': 'false'}.items(),
        condition = UnlessCondition( is_state_pub_disabled )
    )
    # lidar driver
    multiscan_driver = Node(
        name = 'multiscan_driver',
        package = 'multiscan_driver',
        executable = 'multiscan_driver',
        output = 'screen',
        parameters = [
            os.path.join(pkg_path, 'config', 'multiscan_driver.yaml'),
            {
                'driver_hostname': os.popen( 'echo $(hostname -I | awk \'{print $1}\')' ).read().rstrip()
            }
        ],
        remappings = [
            ('lidar_scan', '/multiscan/lidar_scan'),
            ('lidar_imu', '/multiscan/imu')
        ],
        condition = IfCondition( OrSubstitution( is_lidar_driver_forced, is_running_perception ) )
    )
    # perception stack
    perception = Node(
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
        condition = IfCondition( is_running_perception )
    )

#------------------------------------------------------------

    phoenix5_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(phx5_path, 'launch', 'main.launch.py')
        ),
        condition = IfCondition(
            PythonExpression(["'true' if '", phoenix_driver_version, "' == '5' else 'false'"]) )
    )
    phoenix6_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(phx6_path, 'launch', 'main.launch.py')
        ),
        launch_arguments = { 'arduino_device' : arduino_device_id }.items(),
        condition = IfCondition(
            PythonExpression(["'true' if '", phoenix_driver_version, "' == '6' else 'false'"]) )
    )

    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_path, 'launch', 'main.launch.py')
        ),
        condition = IfCondition( is_running_controller )
    )

    # bag2 record (motors)
    motor_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'record_motor.launch.py')
        ),
        condition = IfCondition( is_logging_motor )
    )
    # lidar data
    lidar_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'record_lidar.launch.py')
        ),
        condition = IfCondition( is_logging_lidar )
    )

    # foxglove server if enabled
    foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments =
            {
                'use_sim_time': 'false',
                'mode' : foxglove_config_mode
                # 'mode' : 'test'
            }.items(),
        condition = IfCondition( is_running_foxglove_bridge )
    )

    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('foxglove_mode', default_value='live'),
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('force_lidar_driver', default_value='false'),
        DeclareLaunchArgument('disable_state_pub', default_value='false'),
        DeclareLaunchArgument('record_motor', default_value='false'),
        DeclareLaunchArgument('record_lidar', default_value='false'),
        DeclareLaunchArgument('phoenix_driver', default_value='6'),
        DeclareLaunchArgument('controller', default_value='true'),
        DeclareLaunchArgument("arduino_device", default_value="/dev/ttyACM0"),
        robot_state_publisher,
        multiscan_driver,
        perception,
        phoenix5_driver,
        phoenix6_driver,
        controller_node,
        motor_recorder,
        lidar_recorder,
        foxglove_bridge
    ])
