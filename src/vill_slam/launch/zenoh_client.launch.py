#!/usr/bin/env python3
"""
Zenoh Client Launch File
로봇 ROS2 토픽 → Zenoh → 외부 PC UI 브릿지
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vill_slam_pkg = get_package_share_directory('vill_slam')

    zenoh_config = os.path.join(vill_slam_pkg, 'config', 'zenoh_config.yaml')

    zenoh_client_node = Node(
        package='vill_slam',
        executable='zenoh_client.py',
        name='zenoh_client',
        output='screen',
        parameters=[zenoh_config],
        emulate_tty=True,
    )

    return LaunchDescription([
        zenoh_client_node,
    ])
