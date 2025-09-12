import os
import re
import json
import netifaces

from ament_index_python.packages import get_package_share_directory


def does_package_exist(pkg):
    '''
    Test if a package can be found using `get_package_share_directory()`
    '''
    try:
        get_package_share_directory(pkg)
        return True
    except:
        return False

def try_load_json(json_path, default_json_path = ''):
    '''
    Attempt to load a json if the path is valid, otherwise fallback to the default.
    If both are invalid, or if parsing fails, a `RuntimeError` is thrown.
    '''
    if not json_path:
        if not default_json_path:
            raise RuntimeError('No JSON file provided.')
        json_path = default_json_path
    try:
        with open(json_path, 'r') as f: json_data = f.read()
    except Exception as e:
        raise RuntimeError(f"JSON file '{json_path}' does not exist or could not be read : {e}")
    try:
        return json.loads(json_data)
    except Exception as e:
        raise RuntimeError(f"Failed to load json data from file '{json_path}' : {e}")

def try_load_json_from_args(launch_args, default_json_path = ''):
    '''
    Search for `json_data` arg, followed by `json_path` arg via CLI args,
    then finally use a default path parameter. Raises a `RuntimeError` if none of these
    options are valid.
    '''
    json_data = launch_args.get('json_data', None)
    if not json_data:
        json_data = try_load_json(launch_args.get('json_path', ''), default_json_path)
    return json_data if json_data else {}

def flatten_dict(d, parent_key='', sep='.'):
    '''
    Flatten a nested parameter dict into a dict with single depth, appending
    namespaces using the '.' syntax. This allows for direct insertion into the
    `Node` class's parameters argument.
    '''
    items = {}
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.update(flatten_dict(v, new_key, sep=sep))
        else:
            items[new_key] = v
    return items

def parse_launch_args(arg_list):
    """
    Parse a list of arguments in the format 'key:=value' into a dictionary.

    Args:
        arg_list (list of str): Arguments like ['key:=value', 'key2:=value2']

    Returns:
        dict: Parsed key:value pairs
    """
    parsed_dict = {}
    for arg in arg_list:
        if ":=" in arg:
            key, value = arg.split(":=", 1)  # Split only on the first occurrence
            parsed_dict[key.strip()] = value.strip()
    return parsed_dict



def get_local_ips():
    ips = []
    for iface in netifaces.interfaces():
        addrs = netifaces.ifaddresses(iface)
        if netifaces.AF_INET in addrs:
            for addr in addrs[netifaces.AF_INET]:
                ips.append(addr['addr'])
    return ips

def get_matched_local_ip(local_ips, remote_ip):
    subnet = '.'.join(remote_ip.split('.')[:3])
    for local_ip in local_ips:
        if local_ip.startswith(subnet):
            return local_ip
    return local_ips[0]

def get_local_iface_ips():
    iface_ips = []
    for iface in netifaces.interfaces():
        addrs = netifaces.ifaddresses(iface)
        if netifaces.AF_INET in addrs:
            for addr in addrs[netifaces.AF_INET]:
                ip = addr['addr']
                if ip:
                    iface_ips.append((iface, ip))
    return iface_ips

def get_matched_local_iface(local_iface_ips, remote_ip):
    subnet = '.'.join(remote_ip.split('.')[:3])
    for local_iface_ip in local_iface_ips:
        if local_iface_ip[1].startswith(subnet):
            return local_iface_ip[0]
    return None

def get_mac_from_arp(ip):
    try:
        output = os.popen(f'arp -n {ip}').read().rstrip()
        for line in output.splitlines():
            if ip in line:
                parts = line.split()
                for part in parts:
                    if ':' in part and len(part.split(':')) == 6:
                        return part
    except Exception:
        pass
    return None

def get_bag_topic_types(bag : str):
    topic_pattern = re.compile(r'Topic:\s+(\S+)\s+\|\s+Type:\s+(\S+)')  # "Parse lines like: '/topic_name [msg_type]'"
    output = os.popen(f'ros2 bag info {bag}').read().rstrip()

    type_topics = {}
    for line in output.splitlines():
        match = topic_pattern.search(line)
        if match:
            topic, msg_type = match.groups()
            if msg_type not in type_topics:
                type_topics[msg_type] = []
            type_topics[msg_type].append(topic)

    return type_topics
