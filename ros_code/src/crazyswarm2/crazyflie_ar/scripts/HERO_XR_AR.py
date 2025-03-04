#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import os
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from crazyflie_interfaces.srv import Takeoff, GoTo
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import time

# Packages for path planning
from pathfinding3d.core.diagonal_movement import DiagonalMovement
from pathfinding3d.core.grid import Grid
from pathfinding3d.finder.a_star import AStarFinder
import open3d as o3d
from mpl_toolkits.mplot3d import Axes3D
from PathFinder import PathFinder
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

#collision avoidance
from motion_capture_tracking_interfaces.msg import NamedPose, NamedPoseArray
#from CollisionAvoidance import CollisionAvoidance 


class HeroXR(Node):
    
    def __init__(self, drone_name):
        super().__init__(f'hero_xr_{drone_name}')
        self.get_logger().info('Initializing HeroXR control node')
        
        # Initialize clients and subscriptions for each drone
        self.drone_name = drone_name
        self.drone_position = [0, 0, 0]
        self.get_logger().info(f'Drone position: {self.drone_position}')

        #collision avoidance init
        # self.collision_avoidance = CollisionAvoidance(
        #     ellipsoid_radii=np.array([0.3, 0.3, 0.5]),
        #     max_speed=1,
        #     sidestep_threshold=1.3,
        #     bbox_min=np.array([-10.0, -10.0, 0.0]),
        #     bbox_max=np.array([10.0, 10.0, 5.0])
        # )        

        self.other_positions = {}  # store other drones position

        self.drone_clients = {
            'takeoff_client': self.create_client(Takeoff, f'/{drone_name}/takeoff'),
            'goto_client': self.create_client(GoTo, f'/{drone_name}/go_to')
        }

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(Empty, f'/{drone_name}/takeoff', self.takeoff_callback, 10)
        self.create_subscription(Empty, f'/{drone_name}/land',   self.land_callback,   10)
        self.create_subscription(Pose,  f'/{drone_name}/go_to',  self.goto_callback,   10)
        self.create_subscription(
            PoseStamped,
            f'/{drone_name}/pose',
            self.dronePose_callback,
            qos_profile=qos_profile
        )
        self.get_logger().info('HeroXR node initialized and ready')

        #collision avoidance subscription of /poses
        self.create_subscription(
            NamedPoseArray,       
            "/poses",             
            self.other_dronePose_callback,  
            qos_profile=qos_profile           
        )
        
        #Publisher
        self.waypoints_pub = self.create_publisher(PoseArray, f'/{drone_name}/waypoints', 10)




        # Path-finding setup
        self.grid_unit = 0.1
        x_size = int(6 / self.grid_unit)
        y_size = int(9 / self.grid_unit)
        z_size = int(3 / self.grid_unit)
        self.grid_size = (x_size, y_size, z_size)
        self.x_range = [-2.82, 3.18]
        self.y_range = [-4.86, 4.35]
        self.z_range = [-0.1, 3]
        self.pathfinder = PathFinder(self.grid_unit, resolution=None, grid_size=self.grid_size)
        self.get_logger().info('Path finder initialized and ready')

        # Load point cloud
        pcd = self.pathfinder.load_point_cloud(
            "/home/cpsl/HERO_XR_AR/src/crazyswarm2/crazyflie_ar/point cloud/Vicon1.ply"
        )
        filtered_pcd = self.pathfinder.filter_point_cloud(pcd)
        self.occupancy_map = self.pathfinder.point_cloud_to_numpy(
            filtered_pcd, x_min=-2.82, y_min=-4.86, z_min=0.0
        )

    # ------------- Subscription Callbacks -------------
    def takeoff_callback(self, msg):
        self.get_logger().info(f'Received takeoff command for {self.drone_name}')
        self.takeoff(self.drone_name, 0.5, 3.0)

    def land_callback(self, msg):
        self.get_logger().info(f'Received land command for {self.drone_name}')
        landpoint = [
            float(self.drone_position[0]),
            float(self.drone_position[1]),
            0.0
        ]
        self.land(self.drone_name, landpoint, yaw=0.0, duration=5.0, relative=False)    
            
    def dronePose_callback(self, msg: PoseStamped):
        # Get drone's pose from Vicon system
        self.drone_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

    #collision avoidance
    def other_dronePose_callback(self, msg: NamedPoseArray):
        """
        get all drone positions except itself
        """
        self.other_positions.clear()

        for named_pose in msg.poses:
            if named_pose.name == self.drone_name:
                continue
            x = named_pose.pose.position.x
            y = named_pose.pose.position.y
            z = named_pose.pose.position.z

            # { "cpsl_cf_2": [x, y, z], "cpsl_cf_3": [...], ... }
            self.other_positions[named_pose.name] = [x, y, z]



    #collision avoidance
    def goto_callback(self, msg: Pose):
        goal = [float(msg.position.x),
                float(msg.position.y),
                float(msg.position.z),
        ]
        self.get_logger().info(f'Received go_to command for {self.drone_name}')
        self.get_logger().info(f'Read current pose: {self.drone_position}')
        self.get_logger().info(f'goal: {goal}')

        # Pathfinding
        waypoints = self.pathfinder.find_path(self.occupancy_map, self.drone_position, goal)
        # self.get_logger().info(f'Waypoints: {waypoints}')

        #Publish waypoints
        pose_array = PoseArray()
        pose_array.header.frame_id = 'wolrd'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for waypoint in waypoints:
            pose = Pose()
            pose.position.x = float(waypoint[0])
            pose.position.y = float(waypoint[1])
            pose.position.z = float(waypoint[2])
            pose.orientation.w = 1.0 #default
            pose_array.poses.append(pose)

        self.waypoints_pub.publish(pose_array)
        self.get_logger().info(f'Waypoints published, Length {len(waypoints)}')

         # Send waypoints
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f'Sending waypoint {i+1}/{len(waypoints)} to {self.drone_name}: {waypoint}')
            if not self.is_at_position(self.drone_position, waypoint):
                if self.is_at_position(self.drone_position, waypoint):
                    break
                self.go_to(self.drone_name, waypoint, yaw=0.0, duration=1.0, relative=False)
                time.sleep(0.5)



        # # Send waypoints to drone 1 by 1
        # for i, waypoint in enumerate(waypoints):
        #     self.get_logger().info(
        #         f'Sending waypoint {i + 1}/{len(waypoints)} to {self.drone_name}: {waypoint}'
        #     )

        #     while True:
        #         current_pos = np.array(self.drone_position, dtype=float)

        #         # ---------- collision avoidance ----------
        #         # other positions
        #         neighbors_positions = []
        #         for drone_nm, pos in self.other_positions.items():
        #             neighbors_positions.append(np.array(pos, dtype=float))

        #         adjusted_waypoint = self.collision_avoidance.avoid_collision(
        #             our_position=current_pos,
        #             goal=np.array(waypoint),     # initial point
        #             other_positions=neighbors_positions
        #         )

        #         # project failure or locked
        #         if adjusted_waypoint is None:
        #             self.get_logger().warn(
        #                 f'[Waypoint {i+1}] collisionAvoidance failed. Stopping here.'
        #             )
        #             break

        #         # go_to command with new setpointellipsoid_radii
        #         self.go_to(
        #             drone_name=self.drone_name,
        #             goal=adjusted_waypoint.tolist(),
        #             yaw=0.0,
        #             duration=1.0,
        #             relative=False
        #         )
        #         time.sleep(0.5) 

        #         # ========== is_at_position ==========
        #         # NEED ATTENTION!!! 
        #         # Its a tradeoff to compare is_at_position of adjusted_waypoint or original waypoint!
        #         current_pos = np.array(self.drone_position, dtype=float)
                
        #         if self.is_at_position(current_pos, adjusted_waypoint, tolerance=0.05):
        #             break

        self.get_logger().info(f'All waypoints done for {self.drone_name}')


    # ------------- Service Callers -------------
    def takeoff(self, drone_name, target_height, duration):
        req = Takeoff.Request()
        req.height = target_height
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.group_mask = 0
        self.get_logger().info(f'Sending takeoff request to {drone_name}')
        self.drone_clients['takeoff_client'].call_async(req)

    def go_to(self, drone_name, goal, yaw, duration, relative):
        # Ensure goal is a list or tuple with floats
        if not isinstance(goal, (list, tuple)) or len(goal) != 3:
            raise ValueError(f"Invalid goal format: {goal}. Expected a list or tuple of three floats.")
    
        goal = [float(coord) for coord in goal]  # Ensure all elements are floats
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = goal
        req.yaw = yaw
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.relative = relative
        req.group_mask = 0
        self.get_logger().info(f'Sending go_to request to {drone_name}')
        self.drone_clients['goto_client'].call_async(req)

    def land(self, drone_name, goal, yaw, duration, relative):
        req = GoTo.Request()
        req.goal.x, req.goal.y, req.goal.z = goal
        req.yaw = yaw
        req.duration = rclpy.duration.Duration(seconds=duration).to_msg()
        req.relative = relative
        req.group_mask = 0
        self.get_logger().info(f'Sending go_to request to {drone_name}')
        self.drone_clients['goto_client'].call_async(req)

    # ------------- Utils -------------
    def is_at_position(self, current_position, position, tolerance=0.05):
        return np.linalg.norm(np.array(current_position) - np.array(position)) < tolerance