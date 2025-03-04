#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
import yaml
import os
from ament_index_python.packages import get_package_share_directory

# Import your controller and bridge classes
from HERO_XR_AR import HeroXR  
from Json_ROS import DroneHeadsetBridge

def main(args=None):
    rclpy.init(args=args)

    # Assuming your YAML file is in the config/ directory of this package
    # Get the current directory and construct the full path to .yaml
    package_share_dir = get_package_share_directory('crazyflie_ar')
    yaml_file = os.path.join(package_share_dir, "config", "crazyflies.yaml")

    # Read YAML configuration
    with open(yaml_file, 'r') as f:
        config_data = yaml.safe_load(f)

    # Get the robots section
    robots_section = config_data.get("robots", {})

    # Create multi-threaded executor
    executor = MultiThreadedExecutor()

    # Store created node objects
    drone_nodes = []
    bridge_nodes = []

    # Create nodes for each "drone" in YAML
    for drone_name, info in robots_section.items():
        enabled = info.get("enabled", False)
        if not enabled:
            continue  # Skip disabled drones

        # You can extract uri, initial_position etc., and pass them to HeroXR or bridge nodes if needed
        uri = info.get("uri", "")
        initial_pos = info.get("initial_position", [0.0, 0.0, 0.0])

        # ========== 1) Create controller node ==========
        hero_node = HeroXR(drone_name)  
        drone_nodes.append(hero_node)
        executor.add_node(hero_node)

        # ========== 2) Create bridge node ==========
        bridge_node = DroneHeadsetBridge(drone_name)
        bridge_nodes.append(bridge_node)
        executor.add_node(bridge_node)

    # Start execution
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy all nodes and shut down rclpy
        for node in drone_nodes:
            node.destroy_node()
        for node in bridge_nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
