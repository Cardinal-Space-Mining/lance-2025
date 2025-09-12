import os
import sys

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

PKG_PATH = get_package_share_directory('launch_utils')
sys.path.append(os.path.join(PKG_PATH, 'src'))
from launch_utils.preprocess import preprocess_launch_json
from launch_utils.actions import get_util_actions
from launch_utils.common import try_load_json_from_args, parse_launch_args


def launch(context, *args, **kwargs):
    launch_args = parse_launch_args(context.argv)
    json_data = try_load_json_from_args(launch_args, os.path.join(PKG_PATH, 'config', 'test.json'))
    # print(json_data)
    pp_config = preprocess_launch_json(json_data, launch_args)
    # print(pp_config)
    return get_util_actions(pp_config, launch_args)

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch)
    ])
