#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create Headset Locomotion node
    headset_locomotion_node = Node(
        package='headset_ros_bridge',
        executable='headset_locomotion',
        name='headset_locomotion',
        output='screen'
    )

    # Create Bridge main program node
    headset_bridge_node = Node(
        package='headset_ros_bridge',
        executable='headset_bridge_main',  # Execute the main program
        name='headset_bridge_main',
        output='screen'
    )

    # Return launch description
    return LaunchDescription([
        headset_locomotion_node,
        headset_bridge_node
    ])