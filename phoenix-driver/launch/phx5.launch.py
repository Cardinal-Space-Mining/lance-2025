from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            name = 'phx5_driver',
            package = 'phoenix_ros_driver',
            executable = 'phx5_driver',
            output = 'screen',
            # parameters = []
        )
    ])
