import os
import subprocess
import sys
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time
import logging
from langchain_openai import ChatOpenAI
from rosa import ROSA
from langchain.agents import tool

# Set up logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

def execute_ros_command(command: str) -> tuple[bool, str]:
    """
    Execute a ROS2 command and return success status with output.

    :param command: The ROS2 command to execute.
    :return: A tuple containing a boolean indicating success and the output of the command.
    """
    logger.debug(f"Executing ROS2 command: {command}")
    
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
        logger.debug(f"Command output: {output}")
        return True, output
    except Exception as e:
        logger.error(f"Error executing command: {e}")
        return False, str(e)

##########################################################################################################
# Movement functions

@tool
def crazyflie_move(drone_id: str, dx: float, dy: float, dz: float, timeout: float = 1.0) -> dict:
    """
    Moves the specified Crazyflie drone by given local offsets (dx, dy, dz).

    :param drone_id: Drone identifier.
    :param dx: Local offset in the x-direction (body frame).
    :param dy: Local offset in the y-direction (body frame).
    :param dz: Local offset in the z-direction (body frame).
    :param timeout: Timeout for fetching the drone's current pose.
    :return: A dictionary indicating success or failure.
    """
    logger.debug(f"Moving drone {drone_id} by (dx={dx}, dy={dy}, dz={dz}) with timeout {timeout}")

    echo_cmd = f"ros2 topic echo /{drone_id}/pose --once --spin-time {timeout}"
    success, output = execute_ros_command(echo_cmd)
    if not success:
        return {"error": output}

    try:
        docs = list(yaml.safe_load_all(output))
    except Exception as e:
        logger.error(f"Error parsing YAML output: {e}")
        return {"error": f"YAML parsing error: {e}"}

    data = None
    for doc in docs:
        if doc and "pose" in doc and "position" in doc["pose"]:
            data = doc
            break
    if data is None:
        logger.warning(f"No valid pose found in output: {output}")
        return {"error": f"Could not find valid pose in output: {output}"}

    position = data["pose"]["position"]
    orientation = data["pose"]["orientation"]
    x, y, z = position["x"], position["y"], position["z"]
    qx, qy, qz, qw = orientation["x"], orientation["y"], orientation["z"], orientation["w"]

    if None in [x, y, z, qx, qy, qz, qw]:
        logger.warning(f"Incomplete pose data: {data['pose']}")
        return {"error": f"Pose data incomplete: {data['pose']}"}

    # Rotate local offset into the global frame using quaternion rotation
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

    logger.info(f"Drone {drone_id} moved to new position: (x={new_x:.2f}, y={new_y:.2f}, z={new_z:.2f})")
    return {
        "success": (
            f"{drone_id} moved from (x={x:.2f}, y={y:.2f}, z={z:.2f}) to "
            f"(x={new_x:.2f}, y={new_y:.2f}, z={new_z:.2f})"
        )
    }

# Define specific movement directions using the common `crazyflie_move` function

@tool
def crazyflie_move_forward(drone_id: str = "cpsl_cf_1", distance: float = 1.0, timeout: float = 1.0) -> dict:
    """
    Move the Crazyflie drone forward by a given distance along the local x-axis.

    :param drone_id: The drone identifier.
    :param distance: The distance to move forward (in local body frame).
    :param timeout: Timeout for fetching the drone's current pose.
    :return: A dictionary indicating success or failure.
    """
    logger.debug(f"Moving {drone_id} forward by {distance} meters.")
    return crazyflie_move(drone_id, dx=distance, dy=0.0, dz=0.0, timeout=timeout)

@tool
def crazyflie_move_back(drone_id: str = "cpsl_cf_1", distance: float = 1.0, timeout: float = 1.0) -> dict:
    """
    Move the Crazyflie drone backward by a given distance along the local x-axis.

    :param drone_id: The drone identifier.
    :param distance: The distance to move backward (in local body frame).
    :param timeout: Timeout for fetching the drone's current pose.
    :return: A dictionary indicating success or failure.
    """
    logger.debug(f"Moving {drone_id} backward by {distance} meters.")
    return crazyflie_move(drone_id, dx=-distance, dy=0.0, dz=0.0, timeout=timeout)

@tool
def crazyflie_move_right(drone_id: str = "cpsl_cf_1", distance: float = 1.0, timeout: float = 1.0) -> dict:
    """
    Move the Crazyflie drone right by a given distance along the local y-axis.

    :param drone_id: The drone identifier.
    :param distance: The distance to move right (in local body frame).
    :param timeout: Timeout for fetching the drone's current pose.
    :return: A dictionary indicating success or failure.
    """
    logger.debug(f"Moving {drone_id} right by {distance} meters.")
    return crazyflie_move(drone_id, dx=0.0, dy=distance, dz=0.0, timeout=timeout)

@tool
def crazyflie_move_left(drone_id: str = "cpsl_cf_1", distance: float = 1.0, timeout: float = 1.0) -> dict:
    """
    Move the Crazyflie drone left by a given distance along the local y-axis.

    :param drone_id: The drone identifier.
    :param distance: The distance to move left (in local body frame).
    :param timeout: Timeout for fetching the drone's current pose.
    :return: A dictionary indicating success or failure.
    """
    logger.debug(f"Moving {drone_id} left by {distance} meters.")
    return crazyflie_move(drone_id, dx=0.0, dy=-distance, dz=0.0, timeout=timeout)

@tool
def crazyflie_move_up(drone_id: str = "cpsl_cf_1", distance: float = 0.5, timeout: float = 1.0) -> dict:
    """
    Move the Crazyflie drone upward by a given distance along the local z-axis.

    :param drone_id: The drone identifier.
    :param distance: The distance to move up (in local body frame).
    :param timeout: Timeout for fetching the drone's current pose.
    :return: A dictionary indicating success or failure.
    """
    logger.debug(f"Moving {drone_id} up by {distance} meters.")
    return crazyflie_move(drone_id, dx=0.0, dy=0.0, dz=distance, timeout=timeout)

@tool
def crazyflie_move_down(drone_id: str = "cpsl_cf_1", distance: float = 0.5, timeout: float = 1.0) -> dict:
    """
    Move the Crazyflie drone downward by a given distance along the local z-axis.

    :param drone_id: The drone identifier.
    :param distance: The distance to move down (in local body frame).
    :param timeout: Timeout for fetching the drone's current pose.
    :return: A dictionary indicating success or failure.
    """
    logger.debug(f"Moving {drone_id} down by {distance} meters.")
    return crazyflie_move(drone_id, dx=0.0, dy=0.0, dz=-distance, timeout=timeout)

# Takeoff and land functions

@tool
def crazyflie_takeoff(drone_id: str = "cpsl_cf_1") -> str:
    """
    Command the Crazyflie drone to take off.

    :param drone_id: The drone identifier.
    :return: A success message or error if the takeoff fails.
    """
    logger.debug(f"Sending takeoff command to {drone_id}.")
    try:
        result = subprocess.run(
            ["ros2", "topic", "pub", "--once", f"/{drone_id}/takeoff", "std_msgs/Empty", "{}"],
            check=True,
            capture_output=True,
            text=True
        )
        logger.info(f"Takeoff command successful for {drone_id}.")
        return f"Crazyflie {drone_id} takeoff command executed successfully."
    except subprocess.CalledProcessError as e:
        logger.error(f"Error executing takeoff command for {drone_id}: {e.stderr}")
        return f"Error executing takeoff command for {drone_id}: {e.stderr}"

@tool
def crazyflie_land(drone_id: str = "cpsl_cf_1") -> str:
    """
    Command the Crazyflie drone to land.

    :param drone_id: The drone identifier.
    :return: A success message or error if the land fails.
    """
    logger.debug(f"Sending land command to {drone_id}.")
    try:
        result = subprocess.run(
            ["ros2", "topic", "pub", "--once", f"/{drone_id}/land", "std_msgs/Empty", "{}"],
            check=True,
            capture_output=True,
            text=True
        )
        logger.info(f"Land command successful for {drone_id}.")
        return f"Crazyflie {drone_id} land command executed successfully."
    except subprocess.CalledProcessError as e:
        logger.error(f"Error executing land command for {drone_id}: {e.stderr}")
        return f"Error executing land command for {drone_id}: {e.stderr}"

@tool
def crazyflie_go_to(drone_id: str = "cpsl_cf_1", x: float = 0.0, y: float = 0.0, z: float = 0.0) -> str:
    """
    Command the Crazyflie drone to move to a specified global position.

    :param drone_id: The drone identifier.
    :param x: The target x position in the global coordinate frame.
    :param y: The target y position in the global coordinate frame.
    :param z: The target z position in the global coordinate frame.
    :return: A success message or error if the movement fails.
    """
    logger.debug(f"Sending go_to command to {drone_id} for position ({x}, {y}, {z}).")
    pose_msg = f"{{position: {{x: {x}, y: {y}, z: {z}}}}}"
    try:
        result = subprocess.run(
            ["ros2", "topic", "pub", "--once", f"/{drone_id}/go_to", "geometry_msgs/Pose", pose_msg],
            check=True,
            capture_output=True,
            text=True
        )
        logger.info(f"Go_to command successful for {drone_id}: Moving to ({x}, {y}, {z}).")
        return f"Crazyflie {drone_id} go_to command executed successfully: moving to ({x}, {y}, {z})."
    except subprocess.CalledProcessError as e:
        logger.error(f"Error executing go_to command for {drone_id}: {e.stderr}")
        return f"Error executing go_to command for {drone_id}: {e.stderr}"

###################################################################################################

# ChatOpenAI instance (API key setup by environment variable)
openai_llm = ChatOpenAI(
    model_name="gpt-4o",
    temperature=0,
    max_tokens=500,
    timeout=None,
    max_retries=2,
    openai_api_key=os.getenv("OPEN_AI_KEY")
)

# ROSA instance, passing the movement functions
agent = ROSA(ros_version=2, llm=openai_llm, tools=[
    crazyflie_move_forward, crazyflie_move_back, crazyflie_move_left, crazyflie_move_right,
    crazyflie_move_up, crazyflie_move_down, crazyflie_takeoff, crazyflie_land, crazyflie_go_to
])

### Now you can use natural language to control the drone ###

if len(sys.argv) > 1:
    command = " ".join(sys.argv[1:])
else:
    command = input("Enter command: ")

logger.debug(f"User command: {command}")

result = agent.invoke(command)

logger.info(f"Command result: {result}")
print(result)
