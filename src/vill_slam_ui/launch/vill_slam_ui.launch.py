"""
VILL-SLAM UI Launch File
PyQt5 Monitoring Interface
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('vill_slam_ui')
    config_file = os.path.join(pkg_dir, 'config', 'ui_config.yaml')

    # Launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to UI configuration file'
    )

    fullscreen_arg = DeclareLaunchArgument(
        'fullscreen',
        default_value='false',
        description='Start UI in fullscreen mode'
    )

    # UI node
    ui_node = Node(
        package='vill_slam_ui',
        executable='vill_slam_monitor.py',
        name='vill_slam_monitor',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'fullscreen': LaunchConfiguration('fullscreen')
        }]
    )

    return LaunchDescription([
        config_arg,
        fullscreen_arg,
        ui_node
    ])
