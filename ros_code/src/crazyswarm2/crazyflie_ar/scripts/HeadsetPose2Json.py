#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import requests
import json

from crazyflie_interfaces.msg import Status
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class HeadsetViconBridge(Node):
    def __init__(self):
        super().__init__(f"Vicon_headset_bridge")

        # States of headset
        self.headset_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Subscribe to pose  from Vicon
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/vrpn_mocap/cpsl_quest3/pose',
            self.pose_callback,
            qos_profile=qos_profile
        )

        # Timer: ROS -> Headset
        self.timer_ros_to_headset = self.create_timer(0.01, self.ros_to_headset)

        self.get_logger().info(f"[HeadsetPose2Json] Node initialized.")

    # ------------------- callbacks -------------------
    def pose_callback(self, msg: PoseStamped):
        self.headset_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

    # ------------------- ROS -> Headset -------------------
    def ros_to_headset(self):
        """
        Package battery info, current pose, and waypoints as JSON and POST to headset
        """
        if all(x ==0.0 for x in self.headset_pose):
            return
        HeadsetPoseJson = {"position":[self.headset_pose[0],self.headset_pose[1],self.headset_pose[2]],"orientation":[self.headset_pose[3],self.headset_pose[4],self.headset_pose[5],self.headset_pose[6]]}
        url = "http://192.168.0.107:8000/sendheadsetcurrentposevicon"
        try:
            resp = requests.post(url, json=HeadsetPoseJson)
            if resp.status_code == 200:
                self.get_logger().debug("Headset_Pose posted successfully.")
            else:
                self.get_logger().error(f"Headset_Pose Failed to post data: {resp.status_code}")
        except Exception as e:
            self.get_logger().error(f"Headset_Pose Exception: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = HeadsetViconBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()