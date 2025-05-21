import os
import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def make_foxglove_gui_launch(connection, condition):
    return ExecuteProcess(
        cmd = [
            'foxglove-studio',
            f"\"foxglove://open?ds=foxglove-websocket&ds.url=ws://{connection}/\""  # why doesn't this work!?
        ],
        shell = 'true',
        output = 'screen',
        condition = IfCondition( condition ),
        on_exit = Shutdown(reason='gui closed')
    )

def generate_launch_description():

    pkg_path = get_package_share_directory('lance')
    sim_pkg_path = get_package_share_directory('csm_sim')

    using_foxglove_gui = LaunchConfiguration('foxglove_gui', default='true')
    using_local_foxglove_bridge = LaunchConfiguration('foxglove_bridge', default='false')


    # joystick publisher
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node'
    )

    robot_status_node = Node(
        package = 'lance',
        executable = 'robot_status'
    )

    # publish robot transforms locally to be able to see collada model in foxglove
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_path, 'launch', 'robot_state_publisher.launch.py')
        ),
        condition = IfCondition( LaunchConfiguration('enable_state_pub', default='false') )
    )

    # foxglove bridge
    foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'foxglove.launch.py')
        ),
        launch_arguments =
            {
                'use_sim_time': 'false',
                'mode' : LaunchConfiguration('foxglove_bridge_mode', default='live')
                # 'mode' : 'test'
            }.items(),
        condition = IfCondition( using_local_foxglove_bridge )
    )

    # record motor data locally
    motor_recorder = ExecuteProcess(
        cmd = [
            'ros2', 'bag', 'record',
            '-o', f"bag_recordings/lance_motor_data_{ datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S") }",
            '/joy',
            '/lance/watchdog_status',
            '/lance/track_left/ctrl',
            '/lance/track_left/faults',
            '/lance/track_left/info',
            '/lance/track_right/ctrl',
            '/lance/track_right/faults',
            '/lance/track_right/info',
            '/lance/trencher/ctrl',
            '/lance/trencher/faults',
            '/lance/trencher/info',
            '/lance/hopper_belt/ctrl',
            '/lance/hopper_belt/faults',
            '/lance/hopper_belt/info',
            '/lance/hopper_act/ctrl',
            '/lance/hopper_act/faults',
            '/lance/hopper_act/info',
            '/rosout'
            # '--compression-mode', 'file',
            # '--compression-format', 'zstd'
        ],
        output='screen',
        condition = IfCondition( LaunchConfiguration('record_motor', default='false') )
    )

    # launch foxglove gui (local connection)
    foxglove_gui_local = make_foxglove_gui_launch(
                            'localhost:8765',
                            AndSubstitution(using_foxglove_gui, using_local_foxglove_bridge) )
    # launch foxglove gui (remote connection)
    foxglove_gui_remote = make_foxglove_gui_launch(
                            'mochapanda.local:8765',
                            AndSubstitution(using_foxglove_gui, NotSubstitution(using_local_foxglove_bridge)) )

    return LaunchDescription([
        DeclareLaunchArgument('foxglove_gui', default_value='true'),
        DeclareLaunchArgument('foxglove_bridge', default_value='false'),
        DeclareLaunchArgument('foxglove_bridge_mode', default_value='live'),
        DeclareLaunchArgument('enable_state_pub', default_value='false'),
        DeclareLaunchArgument('record_motor', default_value='false'),
        joy_node,
        robot_status_node,
        robot_state_publisher,
        foxglove_bridge,
        motor_recorder,
        foxglove_gui_local,
        foxglove_gui_remote
    ])
