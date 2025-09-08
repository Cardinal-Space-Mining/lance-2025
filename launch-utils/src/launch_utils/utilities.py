import os
import sys
import json
import yaml
import math
import collections
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
from launch_utils import common, tf_converter


def get_fg_bridge_action(config):
    return Node(
        name = 'fg_bridge',
        package = 'foxglove_bridge',
        executable = 'foxglove_bridge',
        output = 'screen',
        parameters = [config]
    )

def get_fg_gui_action(connection):
    return ExecuteProcess(
        cmd = [
            'foxglove-studio',
            '--url',
            f'"foxglove://open?ds=foxglove-websocket&ds.url=ws://{connection}/"'
        ],
        output = 'screen'
    )
def get_xdg_fg_gui_action(connection):
    return ExecuteProcess(
        cmd = [
            'xdg-open',
            f'foxglove://open?ds=foxglove-websocket&ds.url=ws://{connection}/'
        ],
        output = 'screen'
    )

def get_joy_node_action(config):
    return Node(
        name = 'joy_node',
        package = 'joy',
        executable = 'joy_node',
        parameters = [config]
    )

def get_robot_state_pub_action_from_urdf(urdf):
    return get_robot_state_pub_action({'robot_description' : urdf})
def get_robot_state_pub_action(config):
    return Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [config]
    )

def get_bag_play_action():
    return None

def get_bag_record_action():
    return None


def extract_util_configs(config):
    v = {}
    for tag in ['urdf', 'fg_bridge', 'joy']:
        if tag in config:
            v[tag] = config[tag]
            del config[tag]

def get_util_actions(config):
    a = []
    if 'urdf' in config:
        a.append(
            get_robot_state_pub_action_from_urdf(
                tf_converter.json_to_urdf(config['urdf'])
            )
        )
    if 'fg_bridge' in config:
        a.append(
            get_fg_bridge_action(
                common.flatten_dict(config['fg_bridge'])
            )
        )
    if 'joy' in config:
        a.append(
            get_joy_node_action(
                common.flatten_dict(config['joy'])
            )
        )
    return a
