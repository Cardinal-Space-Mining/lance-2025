import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

PKG_PATH = get_package_share_directory('launch_utils')
sys.path.append(os.path.join(PKG_PATH, 'src'))
from launch_utils.preprocess import preprocess_launch_json
from launch_utils.common import try_load_json, parse_launch_args
from launch_utils import utilities as lu


def launch(context, *args, **kwargs):
    # print(context.argv)
    # print(parse_launch_args(context.argv))

    json_data = try_load_json(os.path.join(PKG_PATH, 'config', 'test.json'))
    # print(json_data)
    pp_config = preprocess_launch_json(json_data, parse_launch_args(context.argv))
    print(pp_config)

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch)
    ])
