"""
VILL-SLAM Main Launch File
Launches complete VILL-SLAM system with line laser and surface processing
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    vill_slam_pkg = get_package_share_directory('vill_slam')
    line_laser_pkg = get_package_share_directory('line_laser_driver')

    # Default config file
    default_config = os.path.join(vill_slam_pkg, 'config', 'vill_slam_params.yaml')

    # Declare launch arguments
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='auto',
        choices=['auto', 'corridor', 'pipe'],
        description='Environment mode: auto, corridor, or pipe'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to VILL-SLAM configuration file'
    )

    enable_line_laser_arg = DeclareLaunchArgument(
        'enable_line_laser',
        default_value='true',
        description='Enable line laser profiler'
    )

    enable_dense_mapping_arg = DeclareLaunchArgument(
        'enable_dense_mapping',
        default_value='true',
        description='Enable dense RGB-D mapping'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # VILL-SLAM Core Node
    vill_slam_node = Node(
        package='vill_slam',
        executable='vill_slam_node',
        name='vill_slam_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Dense Mapper Node
    dense_mapper_node = Node(
        package='vill_slam',
        executable='dense_mapper_node',
        name='dense_mapper_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_dense_mapping')),
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Line Laser Driver (from line_laser_driver package)
    line_laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(line_laser_pkg, 'launch', 'line_laser.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_line_laser')),
        launch_arguments={
            'simulation': PythonExpression(["'", LaunchConfiguration('use_sim_time'), "' == 'true'"])
        }.items()
    )

    return LaunchDescription([
        environment_arg,
        config_file_arg,
        enable_line_laser_arg,
        enable_dense_mapping_arg,
        use_sim_time_arg,
        vill_slam_node,
        dense_mapper_node,
        line_laser_launch
    ])
