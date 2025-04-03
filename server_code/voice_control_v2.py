
########################################################################
import os
import subprocess
from langchain_openai import ChatOpenAI
from rosa import ROSA  
from langchain.agents import tool
import sys
from geometry_msgs.msg import Pose



import yaml  # 引入 PyYAML
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time

def execute_ros_command(command: str) -> tuple[bool, str]:
    """
    Execute a ROS2 command.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """

    # Validate the command is a proper ROS2 command
    cmd = command.split(" ")
    valid_ros2_commands = ["node", "topic", "service", "param", "doctor"]

    if len(cmd) < 2:
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[0] != "ros2":
        raise ValueError(f"'{command}' is not a valid ROS2 command.")
    if cmd[1] not in valid_ros2_commands:
        raise ValueError(f"'ros2 {cmd[1]}' is not a valid ros2 subcommand.")

    try:
        output = subprocess.check_output(command, shell=True).decode()
        return True, output
    except Exception as e:
        return False, str(e)

##########################################################################################################
# Direction movement

@tool
def crazyflie_move_forward(drone_id: str = "cf_1", distance: float = 1.0, timeout: float = 1.0) -> dict:
    """
    Moves the specified Crazyflie drone **forward** by a given distance,
    relative to its current orientation (i.e., "forward" means along its local front direction).

    This uses the drone's current pose and orientation to compute the global target position.

    :param drone_id: Drone identifier (e.g., "cf_1").
    :param distance: Distance to move forward (in local body frame).
    :param timeout: Maximum time to wait for pose information.
    """
    return _crazyflie_relative_move(drone_id, dx=distance, dy=0.0, dz=0.0, timeout=timeout)


@tool
def crazyflie_move_back(drone_id: str = "cf_1", distance: float = 1.0, timeout: float = 1.0) -> dict:
    """
    Moves the specified Crazyflie drone **backward** by a given distance,
    relative to its current orientation (i.e., opposite to its local front direction).

    :param drone_id: Drone identifier.
    :param distance: Distance to move backward (in local body frame).
    """
    return _crazyflie_relative_move(drone_id, dx=-distance, dy=0.0, dz=0.0, timeout=timeout)


@tool
def crazyflie_move_right(drone_id: str = "cf_1", distance: float = 1.0, timeout: float = 1.0) -> dict:
    """
    Moves the specified Crazyflie drone **to the right**, based on its current orientation.
    This means movement along its local right-hand direction (positive y in body frame).

    :param drone_id: Drone identifier.
    :param distance: Distance to move right (in local body frame).
    """

    return _crazyflie_relative_move(drone_id, dx=0.0, dy=distance, dz=0.0, timeout=timeout)


@tool
def crazyflie_move_left(drone_id: str = "cf_1", distance: float = 1.0, timeout: float = 1.0) -> dict:
    """
    Moves the specified Crazyflie drone **to the left**, based on its current orientation.
    This means movement along its local left-hand direction (negative y in body frame).

    :param drone_id: Drone identifier.
    :param distance: Distance to move left (in local body frame).
    """

    return _crazyflie_relative_move(drone_id, dx=0.0, dy=-distance, dz=0.0, timeout=timeout)


@tool
def crazyflie_move_up(drone_id: str = "cf_1", distance: float = 0.5, timeout: float = 1.0) -> dict:
    """
    Moves the specified Crazyflie drone **upward** along its local up axis (usually aligned with global z).

    This is the only direction that often remains aligned with the global z-axis, unless the drone is heavily rotated.

    :param drone_id: Drone identifier.
    :param distance: Distance to move up.
    """

    return _crazyflie_relative_move(drone_id, dx=0.0, dy=0.0, dz=distance, timeout=timeout)


@tool
def crazyflie_move_down(drone_id: str = "cf_1", distance: float = 0.5, timeout: float = 1.0) -> dict:
    """
    Moves the specified Crazyflie drone **downward** along its local down axis.

    Note: This often corresponds to the global negative z direction, depending on the drone's pitch/roll.

    :param drone_id: Drone identifier.
    :param distance: Distance to move down.
    """

    return _crazyflie_relative_move(drone_id, dx=0.0, dy=0.0, dz=-distance, timeout=timeout)




def _crazyflie_relative_move(drone_id: str, dx: float, dy: float, dz: float, timeout: float) -> dict:
    echo_cmd = f"ros2 topic echo /{drone_id}/pose --once --spin-time {timeout}"
    success, output = execute_ros_command(echo_cmd)
    if not success:
        return {"error": output}
    
    try:
        docs = list(yaml.safe_load_all(output))
    except Exception as e:
        return {"error": f"YAML parsing error: {e}"}

    data = None
    for doc in docs:
        if doc and "pose" in doc and "position" in doc["pose"]:
            data = doc
            break
    if data is None:
        return {"error": f"Could not find valid pose in output: {output}"}

    position = data["pose"]["position"]
    orientation = data["pose"]["orientation"]
    x = position.get("x")
    y = position.get("y")
    z = position.get("z")
    qx = orientation.get("x")
    qy = orientation.get("y")
    qz = orientation.get("z")
    qw = orientation.get("w")

    if None in [x, y, z, qx, qy, qz, qw]:
        return {"error": f"Pose data incomplete: {data['pose']}"}

    # 使用四元数将本地偏移 (dx, dy, dz) 旋转到全局方向
    local_offset = np.array([dx, dy, dz])
    r = Rotation.from_quat([qx, qy, qz, qw])
    world_offset = r.apply(local_offset)

    new_x = x + world_offset[0]
    new_y = y + world_offset[1]
    new_z = z + world_offset[2]

    pose_msg = f"{{position: {{x: {new_x}, y: {new_y}, z: {new_z}}}}}"
    pub_cmd = f"ros2 topic pub --once /{drone_id}/go_to geometry_msgs/Pose \"{pose_msg}\""
    success_pub, output_pub = execute_ros_command(pub_cmd)
    if not success_pub:
        return {"error": output_pub}

    return {
        "success": (
            f"{drone_id} moved from (x={x:.2f}, y={y:.2f}, z={z:.2f}) to "
            f"(x={new_x:.2f}, y={new_y:.2f}, z={new_z:.2f})"
        )
    }





###################################################################################################3
#subprocess


@tool
def crazyflie_takeoff(drone_id: str = "cf_1") -> str:
    """
    Publish a ROS2 command to make the specified crazyflie drone take off.
    Executes:
      ros2 topic pub --once /<drone_id>/takeoff std_msgs/Empty "{}"
    :param drone_id: The drone identifier (e.g., "cf_1" or "cf_2").
    """
    try:
        result = subprocess.run(
            ["ros2", "topic", "pub", "--once", f"/{drone_id}/takeoff", "std_msgs/Empty", "{}"],
            check=True,
            capture_output=True,
            text=True
        )
        return f"Crazyflie {drone_id} takeoff command executed successfully."
    except subprocess.CalledProcessError as e:
        return f"Error executing takeoff command for {drone_id}: {e.stderr}"


@tool
def crazyflie_land(drone_id: str = "cf_1") -> str:
    """
    Publish a ROS2 command to make the specified crazyflie drone land.
    Executes:
      ros2 topic pub --once /<drone_id>/land std_msgs/Empty "{}"
    :param drone_id: The drone identifier.
    """
    try:
        result = subprocess.run(
            ["ros2", "topic", "pub", "--once", f"/{drone_id}/land", "std_msgs/Empty", "{}"],
            check=True,
            capture_output=True,
            text=True
        )
        return f"Crazyflie {drone_id} land command executed successfully."
    except subprocess.CalledProcessError as e:
        return f"Error executing land command for {drone_id}: {e.stderr}"

@tool
def crazyflie_go_to(drone_id: str = "cf_1", x: float = 0.0, y: float = 0.0, z: float = 0.0) -> str:
    """
    Publish a ROS2 command to move the specified crazyflie drone to a setpoint.
    Executes:
      ros2 topic pub --once /<drone_id>/go_to geometry_msgs/Pose "{position: {x: <x>, y: <y>, z: <z>}}"
    :param drone_id: The drone identifier.
    :param x: Target x-coordinate.
    :param y: Target y-coordinate.
    :param z: Target z-coordinate (setpoint altitude).
    """
    pose_msg = f"{{position: {{x: {x}, y: {y}, z: {z}}}}}"
    try:
        result = subprocess.run(
            ["ros2", "topic", "pub", "--once", f"/{drone_id}/go_to", "geometry_msgs/Pose", pose_msg],
            check=True,
            capture_output=True,
            text=True
        )
        return f"Crazyflie {drone_id} go_to command executed successfully: moving to ({x}, {y}, {z})."
    except subprocess.CalledProcessError as e:
        return f"Error executing go_to command for {drone_id}: {e.stderr}"
















# ChatOpenAI instance（API key setup by environment variable）
openai_llm = ChatOpenAI(
    model_name="gpt-4o",  # Choose the model u want
    temperature=0,
    max_tokens=500,
    timeout=None,
    max_retries=2,
    openai_api_key=os.getenv("OPEN_AI_KEY")
)

# ROSA instance, and pass all possible function
agent = ROSA(ros_version=2, llm=openai_llm, tools=[crazyflie_move_forward, crazyflie_move_back, crazyflie_move_left, crazyflie_move_right, crazyflie_move_up, crazyflie_move_down, crazyflie_takeoff, crazyflie_land, crazyflie_go_to])

### Now u can use natural language to control the drone ###

# Hard command
command = "can u let drone 1 go left 1 meter ? "


# Soft command
# import sys
# if len(sys.argv) > 1:
#     command = sys.argv[1]
# else:
#     command = "default command"
if len(sys.argv) > 1:
    command = " ".join(sys.argv[1:])  # Take full command from input arguments
else:
    command = input("Enter command: ")  # Prompt the user for command input

result = agent.invoke(command)
print(result)
