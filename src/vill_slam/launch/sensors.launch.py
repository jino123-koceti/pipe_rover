#!/usr/bin/env python3
"""
VILL-SLAM Sensor Launch File
- Ouster OS1-128 LiDAR
- ZED-X Front Camera
- ZED-X Mini Left/Right Cameras
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    vill_slam_pkg = get_package_share_directory('vill_slam')

    # Launch arguments
    ouster_ip_arg = DeclareLaunchArgument(
        'ouster_ip',
        default_value='169.254.241.98',
        description='Ouster LiDAR IP address'
    )

    # --- Static TF for base_link -> zed_camera_link ---
    static_tf_base_to_zed = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_zed',
        arguments=['--x', '0.3', '--y', '0.0', '--z', '0.2',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'zed_camera_link']
    )

    # --- Static TF for base_link -> ouster_lidar ---
    static_tf_base_to_ouster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_ouster',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.5',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'os_sensor']
    )

    # --- Ouster OS1-128 LiDAR Launch ---
    ouster_params_path = os.path.join(vill_slam_pkg, 'config', 'ouster_params.yaml')
    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch',
                'driver.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': ouster_params_path,
            'viz': 'false',
        }.items()
    )

    # --- ZED-X Front Camera Launch ---
    zed_front_config_path = os.path.join(vill_slam_pkg, 'config', 'zed_front_params.yaml')
    zed_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zed_front',
            'camera_model': 'zedx',
            'ros_params_override_path': zed_front_config_path,
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'serial_number': '40782131',
        }.items()
    )

    # --- ZED-X Mini Left Camera Launch ---
    zed_left_config_path = os.path.join(vill_slam_pkg, 'config', 'zed_side_params.yaml')
    zed_left_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zed_left',
            'camera_model': 'zedxone4k',
            'ros_params_override_path': zed_left_config_path,
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'serial_number': '319430526',
        }.items()
    )

    # --- ZED-X Mini Right Camera Launch ---
    zed_right_config_path = os.path.join(vill_slam_pkg, 'config', 'zed_side_params.yaml')
    zed_right_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'zed_right',
            'camera_model': 'zedxone4k',
            'ros_params_override_path': zed_right_config_path,
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'serial_number': '319647570',
        }.items()
    )

    return LaunchDescription([
        ouster_ip_arg,
        static_tf_base_to_zed,
        static_tf_base_to_ouster,
        ouster_launch,
        zed_front_launch,
        zed_left_launch,
        # zed_right_launch,  # Disabled: camera not connected
    ])
