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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class HeroXR(Node):
    def __init__(self, drone_name):
        super().__init__(f'hero_xr_{drone_name}')
        self.get_logger().info('Initializing HeroXR node')
        
        # Initialize clients and subscriptions for each drone
        self.drone_name = drone_name
        self.drone_position = [0,0,0]
        self.get_logger().info(f'Drone position: {self.drone_position}')

        
        self.drone_clients = {
            'takeoff_client': self.create_client(Takeoff, f'/{drone_name}/takeoff'),
            # 'land_client': self.create_client(Land, f'/{drone_name}/land'),
            'goto_client': self.create_client(GoTo, f'/{drone_name}/go_to')
        }

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Empty, f'/{drone_name}/takeoff', self.takeoff_callback, 10)
        self.create_subscription(Empty, f'/{drone_name}/land', self.land_callback, 10)
        self.create_subscription(Pose, f'/{drone_name}/go_to', self.goto_callback, 10)
        self.create_subscription(PoseStamped, f'/vrpn_mocap/{drone_name}/pose', self.dronePose_callback, qos_profile=qos_profile)
        self.get_logger().info('HeroXR node initialized and ready')

        self.grid_unit = 0.1
        self.resolution = None
        x_size = int(6/self.grid_unit)
        y_size = int(9/self.grid_unit)
        z_size = int(3/self.grid_unit)
        self.grid_size = (x_size, y_size, z_size) 
        self.x_range = [-2.82, 3.18]
        self.y_range = [-4.86, 4.35]
        self.z_range = [-0.1, 3]
        self.pathfinder = PathFinder(self.grid_unit, resolution=None, grid_size=self.grid_size)
        self.get_logger().info('Path finder initialized and ready')

         # Load the point cloud
        pcd = self.pathfinder.load_point_cloud("/home/cpsl/HERO_XR_AR/src/crazyswarm2/crazyflie_AR/point cloud/Vicon1.ply")
        # Filter out points out of range
        filtered_pcd = self.pathfinder.filter_point_cloud(pcd)
        # Translate to occupancy map
        self.occupancy_map = self.pathfinder.point_cloud_to_numpy(filtered_pcd, x_min=-2.82, y_min=-4.86, z_min=0.0)

    def takeoff_callback(self, msg):
        self.get_logger().info(f'Received takeoff command for {self.drone_name}')
        self.takeoff(self.drone_name, 0.5, 3.0)

    def land_callback(self, msg):
        self.get_logger().info(f'Received land command for {self.drone_name}')
        landpoint = [
            self.drone_position[0],
            self.drone_position[1],
            0.0]
        self.go_to(self.drone_name, landpoint, yaw=0.0, duration=5.0, relative=False)

    def dronePose_callback(self, msg: PoseStamped):
        #Get drone's pose from Vicon system message 
        current_position = PoseStamped
        current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z]
        # Update pose of the specific drone
        self.drone_position = current_position


    def goto_callback(self, msg: Pose):
        # Print log
        self.get_logger().info(f'Received go_to command for {self.drone_name}')
    
        # Handle Pose message
        goal = [msg.position.x, msg.position.y, msg.position.z]
        self.get_logger().info(f'Read current pose: {self.drone_position}')
        self.get_logger().info(f'goal: {goal}')

        # Find the path and send waypoints 1 by 1
        waypoints = self.pathfinder.find_path(self.occupancy_map, self.drone_position, goal)
        self.get_logger().info(f'Waypoints: {waypoints}')

        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f'Sending waypoint {i + 1}/{len(waypoints)} to {self.drone_name}: {waypoint}')
                
                
            # Wait until drone get to waypoint
            if not self.is_at_position(self.drone_position, waypoint):
                if self.is_at_position(self.drone_position, waypoint):
                    break
                self.go_to(self.drone_name, waypoint, yaw=0.0, duration=1.0, relative=False)
                time.sleep(0.5)


    def takeoff(self, drone_name, target_height, duration):
        req = Takeoff.Request()
        req.height = target_height
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.group_mask = 0
        self.get_logger().info(f'Sending takeoff request to {drone_name}')
        self.drone_clients['takeoff_client'].call_async(req)

    def land(self, drone_name, goal, yaw, duration, relative):
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = goal
        req.yaw = yaw
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.relative = relative
        req.group_mask = 0
        self.get_logger().info(f'Sending go_to request to {drone_name}')
        self.drone_clients['goto_client'].call_async(req)

    def go_to(self, drone_name, goal, yaw, duration,relative):
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = goal
        req.yaw = yaw
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.relative = relative
        req.group_mask = 0
        self.get_logger().info(f'Sending go_to request to {drone_name}')
        self.drone_clients['goto_client'].call_async(req)

    def is_at_position(self, current_position, position, tolerance=0.05):
        return np.linalg.norm(np.array(current_position) - np.array(position)) < tolerance

class UnityXR(HeroXR):
    def __init__(self,drone_nodes):
        super().__init__('unity_xr')
        #LIBRARY OF ACTIVE CF DRONE NODES 
        self.drone_nodes = drone_nodes
        self.get_logger().info('Initializing UnityXR node')
        # Example of accessing drone nodes for operations
        for drone_node in self.drone_nodes:
            #assert drone(s) are set up or enable drones added dynamically?


            self.get_logger().info(f'Managing drone: {drone_node.drone_name}')
            self.drone_positions = {drone_node.drone_name: [0, 0, 0] } # Initialize positions
            self.create_subscription(
        PoseStamped,
        f'/vrpn_mocap/{drone_node.drone_name}/pose',lambda msg, drone_name=drone_node.drone_name: self.handle_pose_update(drone_name, msg),
        QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        )
        self.get_logger().info('UnityXR initialized and subscribed to all drone positions')

        # Timer to periodically log drone positions
        self.create_timer(5.0, self.drone_position)

    def handle_pose_update(self, drone_name, msg: PoseStamped):
        """
        Handle incoming pose updates by updating the position of the corresponding drone.
        """
        current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.drone_positions[drone_name] = current_position
        self.get_logger().info(f"Updated position for {drone_name}: {current_position}")

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--n_agents", help="Number of agents", default=2, type=int)
    parsed_args = parser.parse_args(args)

    N_AGENTS = parsed_args.n_agents

    rclpy.init(args=args)


    drone_nodes = [HeroXR(f'cpsl_cf_{i}') for i in range(1, N_AGENTS + 1)]
    executor = MultiThreadedExecutor()
    for node in drone_nodes:        
        executor.add_node(node)

 
    #TO DO : CREATE WP NODE AND PASS DRONE NODE

    UnityNode = UnityXR(drone_nodes)

    executor.add_node(UnityNode)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in drone_nodes:
            node.destroy_node()
        UnityNode.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':
    main()