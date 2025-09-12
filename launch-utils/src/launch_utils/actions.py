import os
import sys
import json
import yaml
import math
import collections
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
from launch_utils.common import flatten_dict
from launch_utils.tf_converter import json_to_urdf


class NodeAction:
    '''
    Reads a launch config action block as a ros node configuration, and
    provides helper functions for extracting options/formatting a launchable
    object.
    '''
    NODE_OPTIONS_TAG = 'pragma:node_options'

    @staticmethod
    def remove_node_options(config: dict):
        if NodeAction.NODE_OPTIONS_TAG in config:
            del config[NodeAction.NODE_OPTIONS_TAG]

    def __init__(self, config: dict):
        if NodeAction.NODE_OPTIONS_TAG in config:
            self._options = config.pop(NodeAction.NODE_OPTIONS_TAG)
        else:
            self._options = {}
        if 'remappings' in self._options:
            self._remappings = self._options.pop('remappings')
        else:
            self._remappings = {}
        self._config = config

    @property
    def options(self):
        return self._options

    @property
    def remappings(self):
        return self._remappings

    @property
    def config(self):
        return self._config

    def get_option(self, key, default = None):
        return (self.options[key]
                    if key in self.options and self.options[key]
                    else default)

    def get_flattened_params(self) -> dict:
        return flatten_dict(self.config)

    def format_node(self, package: str, executable: str, **kwargs) -> Node:
        '''
        Create a launchable `Node` from the provided config block and passed parameters.
        Note that any provided `kwargs` may be overridden by the internal options if
        there are conflicting keys.
        '''
        return Node(
            package = package,
            executable = executable,
            remappings = self.remappings.items(),
            parameters = [self.get_flattened_params()],
            **{**kwargs, **self.options}
        )




def get_fg_bridge_action(config):
    return NodeAction(config).format_node(
        package= 'foxglove_bridge',
        executable = 'foxglove_bridge',
        output = 'screen'
    )

def get_direct_fg_gui_action(connection):
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
def get_fg_gui_action(config):
    connection = config.get('connection', 'localhost')
    if config.get('use_xdg', False):
        return get_xdg_fg_gui_action(connection)
    else:
        return get_direct_fg_gui_action(connection)

def get_joy_node_action(config):
    return NodeAction(config).format_node(
        package = 'joy',
        executable = 'joy_node',
        output = 'screen'
    )

def get_robot_state_pub_action(config):
    if 'robot_description' in config:
        # replace json description with urdf string --> already formatted for passing to params
        config['robot_description'] = json_to_urdf(config['robot_description'])
    else:
        config['robot_description'] = {}
    return NodeAction(config).format_node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen'
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
        config.get('remappings', {})
    )

def get_bag_record_action(
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
def get_bag_record_action_from_config(config):
    return get_bag_record_action(
        config.get('topics', None),
        config.get('output_prefix', ''),
        config.get('mcap', True)
    )

def get_bag_rerecord_action(
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
def get_bag_rerecord_action_from_config(bag, config):
    return get_bag_rerecord_action(
        bag,
        config.get('exclude_topics', None),
        config.get('mcap', True),
        config.get('output_bag_name', '')
    )


# ---

def get_util_actions(config, launch_args = {}):
    '''
    Handles starting the following utilities using preset action names and launch args:
    - Foxglove bridge node - requires `foxglove_bridge` config block
    - Foxglove gui - requires `foxglove_gui` config block
    - Ros2 joy node - requires `joy_node` config block
    - Robot state publisher - requires `robot_tf` config block
    - Ros2 bag player - requires `bag` launch arg and uses `bag_play` config block if present
    - Ros2 bag record - requires `bag_record` config block
    - Bag rerecorder - requres `bag` launch arg and `bag_rerecord` config block

    Returns list of launchable items, which can be directly passed to `LaunchDescription`.
    '''
    a = []
    if 'foxglove_bridge' in config:
        a.append(get_fg_bridge_action(config['foxglove_bridge']))
    if 'foxglove_gui' in config:
        a.append(get_fg_gui_action(config['foxglove_gui']))
    if 'joy_node' in config:
        a.append(get_joy_node_action(config['joy_node']))
    if 'robot_tf' in config:
        a.append(get_robot_state_pub_action(config['robot_tf']))
    if 'bag' in launch_args:
        a.append(
            get_bag_play_action_from_config(
                launch_args['bag'],
                config.get('bag_play', {})
            )
        )
        if 'bag_rerecord' in config:
            a.append(
                get_bag_rerecord_action_from_config(
                    launch_args['bag'],
                    config['bag_rerecord']
                )
            )
    if 'bag_record' in config:
        a.append(get_bag_record_action_from_config(config['bag_record']))
    return a

def extract_util_configs(config):
    '''
    Removes the following preset action blocks from the config and returns them as a separate dict:
    - `foxglove_bridge`
    - `foxglove_gui`
    - `joy_node`
    - `robot_tf`
    - `bag_play`
    - `bag_record`
    - `bag_rerecord`
    '''
    v = { tag: config[tag] for tag in config if tag in
            [
                'foxglove_bridge',
                'foxglove_gui',
                'joy_node',
                'robot_tf',
                'bag_play',
                'bag_record',
                'bag_rerecord'
            ] }
    for key in v: del config[key]
    return v
