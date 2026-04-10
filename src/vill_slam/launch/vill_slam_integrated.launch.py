"""
VILL-SLAM Integrated Launch File
Launches full system: Sensors + COIN-LIO + VILL-SLAM + Line Laser + Recorder + UI

Usage:
    # Corridor mode (default)
    ros2 launch vill_slam vill_slam_integrated.launch.py

    # Pipe mode
    ros2 launch vill_slam vill_slam_integrated.launch.py environment:=pipe

    # Without sensors (e.g., using rosbag)
    ros2 launch vill_slam vill_slam_integrated.launch.py launch_sensors:=false use_sim_time:=true

    # Minimal (no UI, no recorder)
    ros2 launch vill_slam vill_slam_integrated.launch.py use_ui:=false use_recorder:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package directories
    vill_slam_dir = get_package_share_directory('vill_slam')
    coin_lio_dir = get_package_share_directory('coin_lio')

    # Launch arguments
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='corridor',
        choices=['corridor', 'pipe', 'auto'],
        description='Operating environment mode'
    )

    launch_sensors_arg = DeclareLaunchArgument(
        'launch_sensors',
        default_value='true',
        description='Launch sensor drivers (disable for rosbag playback)'
    )

    use_ui_arg = DeclareLaunchArgument(
        'use_ui',
        default_value='false',
        description='Launch monitoring UI (requires X display - disable for headless/SSH)'
    )

    use_recorder_arg = DeclareLaunchArgument(
        'use_recorder',
        default_value='true',
        description='Launch data recorder'
    )

    auto_record_arg = DeclareLaunchArgument(
        'auto_record',
        default_value='false',
        description='Auto-start recording on launch'
    )

    use_line_laser_arg = DeclareLaunchArgument(
        'use_line_laser',
        default_value='true',
        description='Enable line laser system'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (for rosbag playback)'
    )

    coin_lio_config_arg = DeclareLaunchArgument(
        'coin_lio_config',
        default_value='ouster128.yaml',
        description='COIN-LIO config file name'
    )

    # ============================================================
    # 1. Sensor Drivers (Ouster + ZED cameras + static TFs)
    # ============================================================
    sensors_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_sensors')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vill_slam_dir, 'launch', 'sensors.launch.py')
                )
            )
        ]
    )

    # ============================================================
    # 2. COIN-LIO Odometry (delayed 3s for sensor startup)
    # ============================================================
    coin_lio_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coin_lio_dir, 'launch', 'mapping.launch.py')
                ),
                launch_arguments={
                    'config_file': LaunchConfiguration('coin_lio_config'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'rviz': 'false',
                }.items()
            )
        ]
    )

    # ============================================================
    # 3. VILL-SLAM Core (delayed 5s for COIN-LIO startup)
    # ============================================================
    corridor_config = os.path.join(vill_slam_dir, 'config', 'corridor_mode.yaml')
    pipe_config = os.path.join(vill_slam_dir, 'config', 'pipe_mode.yaml')
    default_config = os.path.join(vill_slam_dir, 'config', 'vill_slam_params.yaml')

    vill_slam_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(vill_slam_dir, 'launch', 'vill_slam.launch.py')
                ),
                launch_arguments={
                    'environment': LaunchConfiguration('environment'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items()
            )
        ]
    )

    # ============================================================
    # 4. Line Laser Driver (optional)
    # ============================================================
    line_laser_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_line_laser')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('line_laser_driver'),
                        'launch', 'line_laser.launch.py')
                )
            )
        ]
    )

    # ============================================================
    # 5. Data Recorder (optional)
    # ============================================================
    recorder_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_recorder')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('vill_slam_recorder'),
                        'launch', 'recorder.launch.py')
                ),
                launch_arguments={
                    'auto_start': LaunchConfiguration('auto_record')
                }.items()
            )
        ]
    )

    # ============================================================
    # 6. Monitoring UI (optional)
    # ============================================================
    ui_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_ui')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('vill_slam_ui'),
                        'launch', 'vill_slam_ui.launch.py')
                )
            )
        ]
    )

    # ============================================================
    # 7. Environment Detector (auto mode only)
    # ============================================================
    environment_detector = Node(
        package='vill_slam',
        executable='environment_detector.py',
        name='environment_detector',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('environment'), "' == 'auto'"])
        )
    )

    return LaunchDescription([
        # Arguments
        environment_arg,
        launch_sensors_arg,
        use_ui_arg,
        use_recorder_arg,
        auto_record_arg,
        use_line_laser_arg,
        use_sim_time_arg,
        coin_lio_config_arg,
        # Launch sequence: sensors → COIN-LIO (3s delay) → VILL-SLAM (5s delay)
        sensors_launch,
        coin_lio_launch,
        vill_slam_launch,
        # Optional components
        line_laser_launch,
        recorder_launch,
        ui_launch,
        environment_detector,
    ])
