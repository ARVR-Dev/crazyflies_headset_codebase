#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import requests
import json

from crazyflie_interfaces.msg import Status
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class DroneHeadsetBridge(Node):
    def __init__(self, drone_name: str):
        super().__init__(f"drone_headset_bridge_{drone_name}")
        self.drone_name = drone_name

        # States of drones
        self.battery_voltage = 0.0
        self.drone_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.waypoints_list = []

        # Subscribe to battery status
        self.status_sub = self.create_subscription(
            Status,
            f'/{self.drone_name}/status',
            self.status_callback,
            10
        )
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to pose  from Vicon
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/vrpn_mocap/{self.drone_name}/pose',
            self.pose_callback,
            qos_profile=qos_profile
        )
        # Subscribe to waypoints from controller
        self.waypoints_sub = self.create_subscription(
            PoseArray,
            f'/{self.drone_name}/waypoints',
            self.waypoints_callback,
            10
        )

        # Publish commands to flight controller
        self.takeoff_pub = self.create_publisher(
            Empty,
            f'/{drone_name}/takeoff',
            10
        )
        self.land_pub = self.create_publisher(
            Empty,
            f'/{drone_name}/land',
            10
        )
        self.go_to_pub = self.create_publisher(
            Pose,
            f'/{self.drone_name}/go_to',
            10
        )

        # Timer: ROS -> Headset
        self.timer_ros_to_headset = self.create_timer(0.01, self.ros_to_headset)
        # Timer: Headset -> ROS
        self.timer_headset_to_ros = self.create_timer(0.1, self.headset_to_ros)

        # Last target point
        self.last_target = None

        self.get_logger().info(f"[DroneHeadsetBridge] Node for {self.drone_name} initialized.")

    # ------------------- callbacks -------------------
    def status_callback(self, msg: Status):
        # Map voltage (3.16V-4.2V) to percentage (0-100)
        voltage = msg.battery_voltage
        percentage = max(0.0, min(100.0, (voltage - 3.16) / (4.2 - 3.16) * 100))
        self.battery_voltage = percentage  # Store percentage valuecpsl@cpsl-NUC13ANKi5

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

    def waypoints_callback(self, msg: PoseArray):
        tmp_list = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            tmp_list.append([x, y, z])
        self.waypoints_list = tmp_list

    # ------------------- ROS -> Headset -------------------
    def ros_to_headset(self):
        """
        Package battery info, current pose, and waypoints as JSON and POST to headset
        """
        droneStateJson = {"namespace":self.drone_name, "position":[self.drone_pose[0],self.drone_pose[1],self.drone_pose[2]],"orientation":[self.drone_pose[3],self.drone_pose[4],self.drone_pose[5],self.drone_pose[6]], "trajectory":self.waypoints_list, "batteryPercentage": self.battery_voltage}

        
        url = "http://192.168.0.107:8000/senddronecurrentstate"
        try:
            resp = requests.post(url, json=droneStateJson)
            if resp.status_code == 200:
                self.get_logger().debug("[ROS -> Headset] Data posted successfully.")
            else:
                self.get_logger().error(f"[ROS -> Headset] Failed to post data: {resp.status_code}")
        except Exception as e:
            self.get_logger().error(f"[ROS -> Headset] Exception: {e}")

        # battery_json = {
        #     "Battery_voltage": {
        #         "Name of Drone": self.drone_name,
        #         "Voltage": float(self.battery_voltage)  # or "Percentage"
        #     }cpsl@cpsl-NUC13ANKi5
        # }

        # pose_json = {
        #     "Poses": {
        #         "Name of Drone": self.drone_name,
        #         "X": float(self.drone_pose[0]),
        #         "Y": float(self.drone_pose[1]),
        #         "Z": float(self.drone_pose[2])
        #     }
        # }

        # trajectory_json = {
        #     "Trajectory": {
        #         "Waypoints": self.waypoints_list
        #     }
        # }

        # merged_json = {**battery_json, **pose_json, **trajectory_json}

        # url = "http://192.168.0.119:8000/ros_to_headset"
        # try:
        #     resp = requests.post(url, json=merged_json)
        #     if resp.status_code == 200:
        #         self.get_logger().debug("[ROS -> Headset] Data posted successfully.")
        #     else:
        #         self.get_logger().error(f"[ROS -> Headset] Failed to post data: {resp.status_code}")
        # except Exception as e:
        #     self.get_logger().error(f"[ROS -> Headset] Exception: {e}")

    # ------------------- Headset -> ROS -------------------
    def headset_to_ros(self):
        """
        Only send to ROS when target point changes
        """
            
        url = "http://192.168.0.107:8000/getoldestdronedestination?droneNamespace="+self.drone_name
        try:
            resp = requests.get(url)
            if resp.status_code == 200:
                data = resp.json()
                targetPosition = data["destination"]

                # Handle takeoff and land commands
                if targetPosition == "takeoff":
                    self.takeoff_pub.publish(Empty())
                    self.get_logger().info(f"[Headset -> ROS] Received 'takeoff' command, published to /{self.drone_name}/takeoff")
                    return  # Avoid sending target position update at the same time

                elif targetPosition == "land":
                    self.land_pub.publish(Empty())
                    self.get_logger().info(f"[Headset -> ROS] Received 'land' command, published to /{self.drone_name}/land")
                    return  # Avoid sending target position update at the same time
                
                if(targetPosition[0] != -1000):
                    x = targetPosition[0]
                    y = targetPosition[1]
                    z = targetPosition[2]
                    
                    # Check if new target point is get_target_positiondifferent from last target point
                    current_target = (x, y, z)
                    if self.last_target != current_target:
                        pose_msg = Pose()
                        pose_msg.position.x = x
                        pose_msg.position.y = y
                        pose_msg.position.z = z
                        self.go_to_pub.publish(pose_msg)
                        
                        self.last_target = current_target  # Update last target point
                        
                        self.get_logger().info(
                            f"[Headset -> ROS] New target: ({x}, {y}, {z}), published to /{self.drone_name}/go_to"
                        )
            else:
                self.get_logger().debug(
                    f"[Headset -> ROS] GET target position failed, code: {resp.status_code}"
                )
        except Exception as e:
            self.get_logger().error(f"[Headset -> ROS] Exception: {e}")

