"""
VILL-SLAM Corridor Test Launch File
For testing in building corridors before pipe deployment
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    vill_slam_pkg = get_package_share_directory('vill_slam')

    # Corridor-specific configuration
    corridor_config = os.path.join(vill_slam_pkg, 'config', 'corridor_mode.yaml')

    # Use sim time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Include main launch with corridor config
    vill_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vill_slam_pkg, 'launch', 'vill_slam.launch.py')),
        launch_arguments={
            'environment': 'corridor',
            'config_file': corridor_config,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_line_laser': 'true',
            'enable_dense_mapping': 'true'
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        vill_slam_launch
    ])
