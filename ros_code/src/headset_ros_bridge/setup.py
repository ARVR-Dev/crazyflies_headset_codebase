from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'headset_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # Install launch scripts
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'requests'],
    zip_safe=True,
    maintainer='cpsl',
    maintainer_email='shaocheng.luo@gmail.com',
    description='Bridge between AR headset and ROS 2 for drone control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'headset_bridge_main = headset_ros_bridge.headset_bridge_main:main',
            'headset_locomotion = headset_ros_bridge.headset_locomotion:main',
            'robots_headset_bridge = headset_ros_bridge.robots_headset_bridge:robots_headset_bridge',
        ],
    },
)
