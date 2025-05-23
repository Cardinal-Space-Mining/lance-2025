import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd = [
                'ros2', 'bag', 'record',
                '-o', f"bag_recordings/lance_lidar_data_{
                            datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S") }",
                '/multiscan/lidar_scan',
                '/multiscan/imu',
                # '/cardinal_perception/tags_detections',
                '/tf',
                '/tf_static',
                # '--compression-mode', 'file',
                # '--compression-format', 'zstd'
            ],
            output='screen'
        )
    ])
