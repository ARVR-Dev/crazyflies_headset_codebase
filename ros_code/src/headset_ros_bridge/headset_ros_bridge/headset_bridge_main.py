#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from headset_ros_bridge.robots_headset_bridge import robots_headset_bridge

def main(args=None):
    rclpy.init(args=args)

    # Get the path to the crazyflies.yaml (assuming it's shared with another package)
    config_package = 'headset_ros_bridge'  # <- this is the name of the original package holding the YAML
    package_share_dir = get_package_share_directory(config_package)
    yaml_file = os.path.join(package_share_dir, "config", "crazyflies.yaml") # change the yaml file for different robots

    # Load YAML config
    with open(yaml_file, 'r') as f:
        config_data = yaml.safe_load(f)

    robots_section = config_data.get("robots", {})

    # Set up multithreaded executor
    executor = MultiThreadedExecutor()
    bridge_nodes = []

    for drone_name, info in robots_section.items():
        if not info.get("enabled", False):
            continue  # Skip disabled drones

        node = robots_headset_bridge(drone_name)
        bridge_nodes.append(node)
        executor.add_node(node)

    # Spin all bridge nodes
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in bridge_nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
