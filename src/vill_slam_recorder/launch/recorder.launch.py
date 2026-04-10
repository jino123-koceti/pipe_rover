"""
VILL-SLAM Data Recorder Launch File
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('vill_slam_recorder')
    config_file = os.path.join(pkg_dir, 'config', 'recorder_params.yaml')

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to recorder configuration file'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Auto-start recording on launch'
    )

    # Recorder node
    recorder_node = Node(
        package='vill_slam_recorder',
        executable='multi_format_recorder.py',
        name='multi_format_recorder',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_arg,
        auto_start_arg,
        recorder_node
    ])
