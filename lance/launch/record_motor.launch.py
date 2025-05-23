import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd = [
            'ros2', 'bag', 'record',
            '-o', f"bag_recordings/lance_motor_data_{
                        datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S") }",
            '/joy',
            '/lance/watchdog_status',
            '/lance/control_level',
            '/lance/mining_status',
            '/lance/offload_status',

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
            output='screen'
        )
    ])