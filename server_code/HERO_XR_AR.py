#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose,PoseStamped
from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from enum import Enum
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import argparse
from pathfinding3d.core.diagonal_movement import DiagonalMovement
from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder
import open3d as o3d
from mpl_toolkits.mplot3d import Axes3D
import time
from PathFinder import PathFinder 

class Motors(Enum):
    MOTOR_CLASSIC = 1  # https://store.bitcraze.io/products/4-x-7-mm-dc-motor-pack-for-crazyflie-2 w/ standard props
    MOTOR_UPGRADE = 2  # https://store.bitcraze.io/collections/bundles/products/thrust-upgrade-bundle-for-crazyflie-2-x

class HeroXR(Node):
    def __init__(self, all_drone_names):
        super().__init__('hero_xr')
        self.get_logger().info('Initializing HeroXR node')
        
        # Initialize clients and subscriptions for each drone
        self.drone_clients = {}
        for drone_name in all_drone_names:
            self.drone_positions[drone_name] = [0,0,0]
            self.drone_clients[drone_name] = {
                'takeoff_client': self.create_client(Takeoff, f'/{drone_name}/takeoff'),
                'land_client': self.create_client(Land, f'/{drone_name}/land'),
                'goto_client': self.create_client(GoTo, f'/{drone_name}/go_to')
            }

            self.create_subscription(Empty, f'/{drone_name}/takeoff', self.create_takeoff_callback(drone_name), 10)
            self.create_subscription(Empty, f'/{drone_name}/land', self.create_land_callback(drone_name), 10)
            self.create_subscription(Pose, f'/{drone_name}/go_to', self.create_goto_callback(drone_name), 10)
            self.create_subscription(PoseStamped, f'/vicon/{drone_name}', self.create_dronePose_callback(drone_name), 10)
        self.get_logger().info('HeroXR node initialized and ready')

        self.grid_unit = 0.1
        self.resolution = None
        x_size = int(6/self.grid_unit)
        y_size = int(9/self.grid_unit)
        z_size = int(3/self.grid_unit)
        self.grid_size = (x_size, y_size, z_size) 
        self.x_range = [-2.82, 3.18]
        self.y_range = [-4.86, 4.14]
        self.z_range = [0, 3]
        pathfinder = PathFinder(self.grid_unit, resolution=None, grid_size=self.grid_size)
        self.get_logger().info('Path finder initialized and ready')

         # Load the point cloud
        pcd = pathfinder.load_point_cloud(r"C:\Users\hqjim\Downloads\Vicon1.ply")
        # Filter out points out of range
        filtered_pcd = pathfinder.filter_point_cloud(pcd)
        # Translate to occupancy map
        self.occupancy_map = pathfinder.point_cloud_to_numpy(filtered_pcd, x_min=-2.82, y_min=-4.86, z_min=0.0)

    def create_takeoff_callback(self, drone_name):
        def takeoff_callback(msg):
            self.get_logger().info(f'Received takeoff command for {drone_name}')
            self.takeoff(drone_name, 0.5, 3.0)
        return takeoff_callback

    def create_land_callback(self, drone_name):
        def land_callback(msg):
            self.get_logger().info(f'Received land command for {drone_name}')
            self.land(drone_name, 0.0, 3.0)
        return land_callback
    
    def create_dronePose_callback(self, drone_name):
        def dronePose_callback(msg: PoseStamped):
            #Get drone's pose from Vicon system message 
            current_position = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]
            # Update pose of the specific drone
            self.drone_positions[drone_name] = current_position
        return dronePose_callback

    def create_goto_callback(self, drone_name):
        def goto_callback(msg: Pose):
            self.get_logger().info(f'Received go_to command for {drone_name}')
        
            # Editing target message
            goal = [msg.position.x, msg.position.y, msg.position.z]

            # Using PathFinder(map,current position,target position) to generate the path
            waypoints = PathFinder.find_path(self.occupancy_map,self.drone_positions[drone_name], goal)
        
            # Sending waypoints to drone one by one
            for waypoint in waypoints:
                yaw = 0.0  # Keep the same yaw angle
                duration = 2.0  # Duration of moving point to point
                relative = False
                self.go_to(drone_name, waypoint, yaw, duration, relative)

                # Wait until the drone get close enough to the sent waypoint 
                while not self.is_at_position(self.drone_positions[drone_name], waypoint):  
                    rclpy.spin_once(self)
            return goto_callback

    def takeoff(self, drone_name, target_height, duration):
        req = Takeoff.Request()
        req.height = target_height
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.group_mask = 0
        self.get_logger().info(f'Sending takeoff request to {drone_name}')
        self.drone_clients[drone_name]['takeoff_client'].call_async(req)

    def land(self, drone_name, target_height, duration):
        req = Land.Request()
        req.height = target_height
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.group_mask = 0
        self.get_logger().info(f'Sending land request to {drone_name}')
        self.drone_clients[drone_name]['land_client'].call_async(req)

    def go_to(self, drone_name, goal, yaw, duration,relative):
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = goal
        req.yaw = yaw
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.relative = relative
        req.group_mask = 0
        self.get_logger().info(f'Sending go_to request to {drone_name}')
        self.drone_clients[drone_name]['goto_client'].call_async(req)

    def is_at_position(self, current_position, position, tolerance=0.05):
        return np.linalg.norm(np.array(current_position) - np.array(position)) < tolerance


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--n_agents", help="Number of agents", default=2, type=int)
    parsed_args = parser.parse_args(args)

    N_AGENTS = parsed_args.n_agents

    rclpy.init(args=args)

    drone_names = [f'cf_{i}' for i in range(1, N_AGENTS + 1)]

    hero_xr = HeroXR(drone_names)
    
    executor = MultiThreadedExecutor()
    executor.add_node(hero_xr)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        hero_xr.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
