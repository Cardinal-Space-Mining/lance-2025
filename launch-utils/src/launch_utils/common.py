import os
import json
import netifaces

from ament_index_python.packages import get_package_share_directory


def does_package_exist(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except:
        return False

def try_load_json(json_path, default_json_path = ''):
    if not json_path:
        json_path = default_json_path
    try:
        with open(json_path, 'r') as f: json_data = f.read()
    except Exception as e:
        raise RuntimeError(f"JSON file '{json_path}' does not exist or could not be read : {e}")
    try:
        return json.loads(json_data)
    except Exception as e:
        raise RuntimeError(f"Failed to load json data from file '{json_path}' : {e}")

def flatten_dict(d, parent_key='', sep='.'):
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
