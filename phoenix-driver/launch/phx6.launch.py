from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("arduino_device", default_value="/dev/ttyACM0"),
        Node(
            name = 'phx6_driver',
            package = 'phoenix_ros_driver',
            executable = 'phx6_driver',
            output = 'screen',
            parameters = [
                {'arduino_device': LaunchConfiguration("arduino_device", default="/dev/ttyACM0") }
            ]
        )
    ])
