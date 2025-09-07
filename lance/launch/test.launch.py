import os
import sys
import json
import yaml
import math
import collections
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory('lance'), 'launch'))

from util import print_something

def yaml_to_json():
    yaml_path = os.path.join(get_package_share_directory('lance'), 'config', 'perception_live.yaml')
    try:
        with open(yaml_path, 'r') as f: yaml_data = yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError('error reading yaml file')

    print(json.dumps(yaml_data))

def json_to_yaml():
    json_path = os.path.join(get_package_share_directory('lance'), 'config', 'perception.json')
    try:
        with open(json_path, 'r') as f: json_data = f.read()
    except Exception as e:
        raise RuntimeError('error reading json file')

    print(yaml.dump(json_data))
    print(print_something())

def run_launch(context, *args, **kwargs):
    json_to_yaml()

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=run_launch)
    ])
