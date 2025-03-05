
import os
import subprocess
from langchain_openai import ChatOpenAI
from rosa import ROSA  
from langchain.agents import tool
import sys

@tool
def crazyflie_takeoff(drone_id: str = "cpsl_cf_1") -> str:
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
def crazyflie_land(drone_id: str = "cpsl_cf_1") -> str:
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
def crazyflie_go_to(drone_id: str = "cpsl_cf_1", x: float = 0.0, y: float = 0.0, z: float = 0.0) -> str:
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
    max_tokens=100,
    timeout=None,
    max_retries=2,
    openai_api_key="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"
)
# ROSA instance, and pass all possible function
agent = ROSA(ros_version=2, llm=openai_llm, tools=[crazyflie_takeoff, crazyflie_land, crazyflie_go_to])

### Now u can use natural language to control the drone ###

# Hard command
if len(sys.argv) > 1:
    command = " ".join(sys.argv[1:])  # Take full command from input arguments
else:
    command = input("Enter command: ")  # Prompt the user for command input
# Soft command
# import sys
# if len(sys.argv) > 1:
#     command = sys.argv[1]
# else:
#     command = "default command"
result = agent.invoke(command)
print(result)
