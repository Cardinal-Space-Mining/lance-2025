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

# ----

def get_bag_play_action(
        bag : str,
        topics : list = [],
        paused : bool = True,
        loop : bool = False,
        remappings : dict = {} ):
    cmd_args = ['ros2', 'bag', 'play', '--clock', '10', bag]
    if topics:
        cmd_args.append('--topics')
        cmd_args.extend(topics)
    if paused:
        cmd_args.append('--start-paused')
    if loop:
        cmd_args.append('--loop')
    if remappings:
        cmd_args.append('--remap')
        for in_, out_ in remappings.items():
            cmd_args.append(f'{in_}:={out_}')

    return ExecuteProcess(
        cmd = cmd_args,
        output = 'screen'
    )
def get_bag_play_action_from_config(bag, config):
    return get_bag_play_action(
        bag,
        config.get('topics', []),
        config.get('start_paused', True),
        config.get('loop', False),
        config.get('remappings', {}))
def add_bag_record_action(
        topics : list,
        file_prefix = 'bag_recordings/bag',
        mcap = True ):
    cmd_args = [
        'ros2', 'bag', 'record',
        '-o', f'{file_prefix}_{ datetime.now().strftime("%Y_%m_%d-%H_%M_%S") }' ]
    if mcap:
        cmd_args.append('-s')
        cmd_args.append('mcap')
    if topics:
        cmd_args.extend(topics)
    else:
        cmd_args.append('--all')
    ExecuteProcess(
        cmd = cmd_args,
        output = 'screen'
    )
def add_bag_rerecord_action(
        src_bag : str,
        exclude_topics : list = [],
        mcap = True,
        bag_name = ''):
    cmd_args = [
        'ros2', 'bag', 'record', '--all', '--use-sim-time',
        '-o', (bag_name if bag_name else
                f'{src_bag.rstrip("/")}-rerecord_{ datetime.now().strftime("%Y_%m_%d-%H_%M_%S") }') ]
    if mcap:
        cmd_args.append('-s')
        cmd_args.append('mcap')
    if exclude_topics:
        cmd_args.append('--exclude')
        cmd_args.append('|'.join(exclude_topics))
    ExecuteProcess(
        cmd = cmd_args,
        output = 'screen'
    )


# ---

def extract_util_configs(config):
    v = {}
    for tag in ['urdf', 'fg_bridge', 'joy']:
        if tag in config:
            v[tag] = config[tag]
            del config[tag]

def get_util_actions(config, launch_args = {}):
    a = []
    if 'robot_tf' in config:
        a.append(
            get_robot_state_pub_action_from_urdf(
                tf_converter.json_to_urdf(config['robot_tf'])
            )
        )
    if 'foxglove_bridge' in config:
        a.append(
            get_fg_bridge_action(
                common.flatten_dict(config['foxglove_bridge'])
            )
        )
    if 'joy_node' in config:
        a.append(
            get_joy_node_action(
                common.flatten_dict(config['joy_node'])
            )
        )
    if 'bag' in launch_args:
        a.append(
            get_bag_play_action_from_config(
                launch_args['bag'],
                config.get('bag_play', {})
            )
        )
    return a
