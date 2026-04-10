"""
Line Laser Driver Launch File
Launches GPIO controller, sync node, and processor for VILL-SLAM
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('line_laser_driver')
    config_file = os.path.join(pkg_dir, 'config', 'line_laser_params.yaml')

    # Declare launch arguments
    dual_laser_arg = DeclareLaunchArgument(
        'dual_laser',
        default_value='false',
        description='Enable dual laser mode (120° x 2)'
    )

    sync_mode_arg = DeclareLaunchArgument(
        'sync_mode',
        default_value='alternating',
        description='Synchronization mode: alternating, continuous, manual'
    )

    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode (no GPIO)'
    )

    # GPIO Controller Node
    gpio_controller_node = Node(
        package='line_laser_driver',
        executable='gpio_control.py',
        name='laser_gpio_controller',
        output='screen',
        parameters=[
            config_file,
            {
                'dual_laser_mode': LaunchConfiguration('dual_laser'),
                'simulation_mode': LaunchConfiguration('simulation')
            }
        ]
    )

    # Laser Sync Node
    sync_node = Node(
        package='line_laser_driver',
        executable='laser_sync_node.py',
        name='laser_sync_node',
        output='screen',
        parameters=[
            config_file,
            {
                'sync_mode': LaunchConfiguration('sync_mode')
            }
        ]
    )

    # Line Laser Processor Node
    processor_node = Node(
        package='line_laser_driver',
        executable='line_laser_processor_node',
        name='line_laser_processor',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        dual_laser_arg,
        sync_mode_arg,
        simulation_arg,
        gpio_controller_node,
        sync_node,
        processor_node
    ])
