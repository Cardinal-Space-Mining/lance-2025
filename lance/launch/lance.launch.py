import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

sys.path.append(os.path.join(get_package_share_directory('launch_utils'), 'src'))
from launch_utils.preprocess import preprocess_launch_json
from launch_utils.actions import NodeAction, get_util_actions
from launch_utils.common import try_load_json_from_args, parse_launch_args, get_local_ips, get_matched_local_ip

try:
    sys.path.append(os.path.join(get_package_share_directory('cardinal_perception'), 'launch'))
    from perception_launch_utils import get_perception_actions
    HAVE_PERCEPTION_UTILS = True
except Exception as e:
    HAVE_PERCEPTION_UTILS = False

try:
    sys.path.append(os.path.join(get_package_share_directory('csm_sim'), 'launch'))
    from sim_launch_utils import get_sim_actions
    HAVE_SIM_UTILS = True
except Exception as e:
    HAVE_SIM_UTILS = False

PKG_PATH = get_package_share_directory('lance')
DEFAULT_JSON_PATH = os.path.join(PKG_PATH, 'config', 'lance.json')


def get_multiscan_driver_action(config):
    if 'driver_hostname' not in config and 'lidar_hostname' in config:
        config['driver_hostname'] = get_matched_local_ip(
            get_local_ips(),
            config['lidar_hostname'])
    return NodeAction(config).format_node(
        package = 'multiscan_driver',
        executable = 'multiscan_driver',
        output = 'screen'
    )

def get_phx5_action(config):
    return NodeAction(config).format_node(
        package = 'phoenix_ros_driver',
        executable = 'phx5_driver',
        output = 'screen'
    )

def get_phx6_action(config, launch_args):
    if 'arduino_device' in launch_args:
        config['arduino_device'] = launch_args['arduino_device']
    return NodeAction(config).format_node(
        package = 'phoenix_ros_driver',
        executable = 'phx6_driver',
        output = 'screen'
    )

def get_phx_sim_action(config):
    return NodeAction(config).format_node(
        package = 'phoenix_ros_driver',
        executable = 'motor_sim',
        output = 'screen'
    )

def get_robot_control_action(config):
    return NodeAction(config).format_node(
        package = 'lance',
        executable = 'robot_control',
        output = 'screen'
    )

def get_watchdog_action(config):
    return NodeAction(config).format_node(
        package = 'lance',
        executable = 'robot_status',
        output = 'screen'
    )

def get_robot_actions(config, launch_args = {}):
    a = []
    if 'multiscan_driver' in config:
        a.append(get_multiscan_driver_action(config['multiscan_driver']))
    if 'phoenix5_driver' in config:
        a.append(get_phx5_action(config['phoenix5_driver']))
    if 'phoenix6_driver' in config:
        a.append(get_phx6_action(config['phoenix6_driver'], launch_args))
    if 'phoenix_motor_sim' in config:
        a.append(get_phx_sim_action(config['phoenix_motor_sim']))
    if 'robot_control' in config:
        a.append(get_robot_control_action(config['robot_control']))
    if 'robot_status' in config:
        a.append(get_watchdog_action(config['robot_status']))
    return a


def launch(context, *args, **kwargs):
    actions = []

    launch_args = parse_launch_args(context.argv)
    json_data = try_load_json_from_args(launch_args, DEFAULT_JSON_PATH)
    config = preprocess_launch_json(json_data, launch_args)

    actions.extend(get_util_actions(config, launch_args))
    actions.extend(get_robot_actions(config, launch_args))

    if HAVE_PERCEPTION_UTILS:
        actions.extend(get_perception_actions(config))
    else:
        print("Failed to load 'Cardinal Perception' launch utils!")

    if HAVE_SIM_UTILS:
        actions.extend(get_sim_actions(config))

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch),
    ])
