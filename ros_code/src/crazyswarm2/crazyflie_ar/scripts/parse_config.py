#!/usr/bin/env python3

import yaml
from typing import Dict, Any

def load_yaml(file_path: str) -> Dict[str, Any]:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def get_enabled_drones(config_data: Dict[str, Any]):
    """
    Read the 'robots' field from config_data, filter drones with enabled=True.
    Returns a list of [(drone_name, { 'uri':..., 'initial_position':... }), ... ]
    """
    robots_section = config_data.get("robots", {})
    result = []
    for drone_name, info in robots_section.items():
        if info.get("enabled", False):
            result.append((drone_name, info))
    return result