#!/usr/bin/env python3
"""
Multi-Format Data Recorder for VILL-SLAM
Records CSV, Video, Images based on Scout Mini implementation

Based on: /home/test/ros2_ws/scout_mini/code/catkin_ws/src/data_manager/
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, NavSatFix, Imu, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from vill_slam_msgs.msg import VillSlamStatus, SurfaceSection

# psutil for CPU/MEM monitoring (optional)
try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False

# NOTE: cv_bridge is incompatible with NumPy 2.x on this system
# (cvtColor2 segfaults). Use manual numpy conversion instead.
import cv2
import numpy as np
import csv
import os
import time
from datetime import datetime
import threading
from dataclasses import dataclass, field
from typing import Optional, Dict, Any


@dataclass
class SensorData:
    """Container for synchronized sensor data"""
    timestamp: float = 0.0
    # Wheel odometry
    wheel_x: float = 0.0
    wheel_y: float = 0.0
    linear_vel: float = 0.0
    angular_vel: float = 0.0
    # SLAM pose (vill_slam corrected)
    slam_x: float = 0.0
    slam_y: float = 0.0
    slam_z: float = 0.0
    slam_roll: float = 0.0
    slam_pitch: float = 0.0
    slam_yaw: float = 0.0
    slam_distance: float = 0.0
    # Environmental
    temperature: float = -999.0
    humidity: float = -999.0
    # GPS
    latitude: float = -999.0
    longitude: float = -999.0
    altitude: float = -999.0
    # VILL-SLAM specific
    pipe_radius: float = -999.0
    pipe_eccentricity: float = -999.0
    environment_mode: int = 0


@dataclass
class SlamMetrics:
    """SLAM 성능 분석용 데이터 (주행 후 발산 분석 목적)"""
    timestamp: float = 0.0
    # COIN-LIO raw odometry (보정 전)
    raw_x: float = 0.0
    raw_y: float = 0.0
    raw_z: float = 0.0
    # VILL-SLAM corrected pose (보정 후)
    corr_x: float = 0.0
    corr_y: float = 0.0
    corr_z: float = 0.0
    corr_yaw_deg: float = 0.0
    # 보정량 (norm of difference)
    correction_norm: float = 0.0
    # Twist
    vel_linear: float = 0.0
    vel_angular: float = 0.0

    # ===== 발산 감지 핵심 지표 =====
    # 프레임 간 이동/회전 (점프 감지)
    raw_step: float = 0.0           # 마지막 raw_odom 프레임 사이 이동량 (m)
    corr_step: float = 0.0          # 마지막 corrected pose 프레임 사이 이동량 (m)
    raw_jump: float = 0.0           # 비정상 이동 감지 (>0.5m/frame)
    # 포즈 covariance (불확실성 - 발산 직전 폭증)
    pose_cov_xyz: float = 0.0       # raw odometry trace of position covariance
    pose_cov_rot: float = 0.0       # raw odometry trace of rotation covariance
    # IMU 상태 (발산은 IMU 이상에서 시작되는 경우가 많음)
    imu_acc_x: float = 0.0          # m/s^2
    imu_acc_y: float = 0.0
    imu_acc_z: float = 0.0
    imu_acc_norm: float = 0.0       # |a| (정상: ~9.8)
    imu_gyro_x: float = 0.0         # rad/s
    imu_gyro_y: float = 0.0
    imu_gyro_z: float = 0.0
    imu_gyro_norm: float = 0.0      # |w|
    # LiDAR 환경 인식 (포인트 수 감소 = 특징점 부족)
    lidar_point_count: int = 0
    # ===== 발산 감지 끝 =====

    # VILL-SLAM status
    num_keyframes: int = 0
    num_constraints: int = 0
    loop_closure_count: int = 0
    estimated_drift: float = 0.0
    total_distance: float = 0.0
    environment_mode: int = 0
    pipe_radius: float = -999.0
    pipe_radius_confidence: float = 0.0
    sensors_active: int = 0
    sensors_healthy: int = 0
    # Topic frequencies (recorder가 직접 측정)
    lidar_hz: float = 0.0
    imu_hz: float = 0.0
    cam_front_hz: float = 0.0
    odom_hz: float = 0.0
    # System resources
    cpu_pct: float = -1.0
    mem_pct: float = -1.0


class HzCounter:
    """슬라이딩 윈도우 주파수 측정"""
    def __init__(self, window: float = 2.0):
        self._window = window
        self._times: list = []

    def tick(self):
        now = time.monotonic()
        self._times.append(now)
        cutoff = now - self._window
        self._times = [t for t in self._times if t > cutoff]

    def hz(self) -> float:
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return (len(self._times) - 1) / span if span > 0 else 0.0


class MultiFormatRecorder(Node):
    """Multi-format data recorder (CSV, Video, Images)"""

    def __init__(self):
        super().__init__('multi_format_recorder')

        # Parameters
        self._declare_node_parameters()
        self.load_parameters()

        # State
        self.is_recording = False
        self.current_data = SensorData()
        self.current_metrics = SlamMetrics()
        self.prev_slam_position = None

        # SLAM raw vs corrected pose tracking
        self.raw_pose = None  # COIN-LIO raw odometry (np.ndarray of shape (3,))
        self.corr_pose = None  # VILL-SLAM corrected pose

        # Hz counters for performance monitoring
        self.hz_lidar = HzCounter()
        self.hz_imu = HzCounter()
        self.hz_cam_front = HzCounter()
        self.hz_raw_odom = HzCounter()

        # Video writers
        self.video_writers: Dict[str, cv2.VideoWriter] = {}
        self.last_image_times: Dict[str, float] = {}
        self.measured_fps: Dict[str, float] = {}
        self.frame_counts: Dict[str, int] = {}

        # Image snapshot state
        self.last_snapshot_distance = 0.0
        self.snapshot_interval = self.get_parameter('image_interval_meters').value
        self.snapshot_time_interval = self.get_parameter('image_interval_seconds').value
        self.last_snapshot_time = {}  # per-camera last snapshot wall time

        # CSV file handles
        self.csv_file = None
        self.csv_writer = None
        self.metrics_file = None
        self.metrics_writer = None
        self.loop_file = None
        self.loop_writer = None
        self.last_loop_count = 0

        # psutil process handle
        self.process = psutil.Process() if HAS_PSUTIL else None
        if not HAS_PSUTIL:
            self.get_logger().warn(
                'psutil not available - CPU/MEM monitoring disabled')

        # Locks
        self.data_lock = threading.Lock()
        self.metrics_lock = threading.Lock()

        # Create output directories
        self.create_output_dirs()

        # Subscribers
        self.create_subscribers()

        # Services
        self.create_services()

        # Timer for CSV writing
        csv_interval = self.get_parameter('csv_interval').value
        self.csv_timer = self.create_timer(csv_interval, self.write_csv_row)

        # Timer for metrics CSV (SLAM 분석용)
        metrics_interval = self.get_parameter('metrics_interval').value
        self.metrics_timer = self.create_timer(metrics_interval, self.write_metrics_row)

        self.get_logger().info('Multi-format recorder initialized')
        self.get_logger().info(
            f'  SLAM metrics: {"ENABLED" if self.get_parameter("enable_metrics").value else "DISABLED"}')
        self.get_logger().info(f'  psutil: {"available" if HAS_PSUTIL else "MISSING"}')

    def _declare_node_parameters(self):
        """Declare ROS parameters"""
        # Save paths
        self.declare_parameter('save_base_path', '/mnt/SSD/vill_slam/')
        self.declare_parameter('csv_path', 'csv/')
        self.declare_parameter('video_path', 'video/')
        self.declare_parameter('image_path', 'images/')

        # Recording settings
        self.declare_parameter('csv_interval', 0.1)  # 10 Hz
        self.declare_parameter('image_interval_meters', 1.0)
        # Time-based snapshot fallback (saves an image every N seconds even
        # when slam_distance is stuck at 0). Set to 0 to disable.
        self.declare_parameter('image_interval_seconds', 5.0)
        self.declare_parameter('video_fps', 30)
        self.declare_parameter('video_width', 1920)
        self.declare_parameter('video_height', 1200)
        self.declare_parameter('video_codec', 'mp4v')

        # Topic names
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('slam_odom_topic', '/vill_slam/odometry')
        self.declare_parameter('slam_status_topic', '/vill_slam/status')
        self.declare_parameter('surface_section_topic', '/vill_slam/surface_section')
        self.declare_parameter('temp_humi_topic', '/temp_humi')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('camera_front_topic', '/zed_front/zed_node/rgb/color/rect/image')
        self.declare_parameter('camera_left_topic', '/zed_left/zed_node/rgb/color/rect/image')
        self.declare_parameter('camera_right_topic', '/zed_right/zed_node/rgb/color/rect/image')
        # SLAM analysis topics
        self.declare_parameter('raw_odom_topic', '/Odometry')              # COIN-LIO raw
        self.declare_parameter('lidar_topic', '/ouster/points')            # LiDAR Hz
        self.declare_parameter('imu_topic', '/ouster/imu')                 # IMU Hz
        # Metrics CSV
        self.declare_parameter('enable_metrics', True)
        self.declare_parameter('metrics_interval', 0.1)  # 10 Hz

        # Enable/disable recording types
        self.declare_parameter('enable_csv', True)
        self.declare_parameter('enable_video', True)
        self.declare_parameter('enable_images', True)

    def load_parameters(self):
        """Load parameters into instance variables"""
        self.base_path = self.get_parameter('save_base_path').value
        self.csv_path = os.path.join(self.base_path, self.get_parameter('csv_path').value)
        self.video_path = os.path.join(self.base_path, self.get_parameter('video_path').value)
        self.image_path = os.path.join(self.base_path, self.get_parameter('image_path').value)

        self.video_fps = self.get_parameter('video_fps').value
        self.video_width = self.get_parameter('video_width').value
        self.video_height = self.get_parameter('video_height').value
        self.video_codec = self.get_parameter('video_codec').value

    def create_output_dirs(self):
        """Create output directories if they don't exist"""
        for path in [self.csv_path, self.video_path, self.image_path]:
            os.makedirs(path, exist_ok=True)

    def create_subscribers(self):
        """Create topic subscribers"""
        # Image: SensorDataQoS — ZED 발행 QoS와 매칭
        image_qos = qos_profile_sensor_data

        # Diagnostic: 첫 호출 추적
        self._first_callback_logged = set()

        # Odometry (wheel + SLAM corrected)
        self.wheel_odom_sub = self.create_subscription(
            Odometry, self.get_parameter('wheel_odom_topic').value,
            self.wheel_odom_callback, 10)

        self.slam_odom_sub = self.create_subscription(
            Odometry, self.get_parameter('slam_odom_topic').value,
            self.slam_odom_callback, 10)

        # COIN-LIO raw odometry (SLAM 분석용 — 보정 전 포즈)
        self.raw_odom_sub = self.create_subscription(
            Odometry, self.get_parameter('raw_odom_topic').value,
            self.raw_odom_callback, 10)

        # 토픽 Hz 측정용 구독 (데이터 자체는 안 씀, 단지 주파수 카운팅)
        self.lidar_hz_sub = self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value,
            self._lidar_hz_callback, qos_profile_sensor_data)
        self.imu_hz_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._imu_hz_callback, qos_profile_sensor_data)

        # VILL-SLAM specific
        self.slam_status_sub = self.create_subscription(
            VillSlamStatus, self.get_parameter('slam_status_topic').value,
            self.slam_status_callback, 10)

        self.surface_sub = self.create_subscription(
            SurfaceSection, self.get_parameter('surface_section_topic').value,
            self.surface_callback, 10)

        # Environmental sensors
        self.temp_humi_sub = self.create_subscription(
            PointStamped, self.get_parameter('temp_humi_topic').value,
            self.temp_humi_callback, 10)

        self.gps_sub = self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value,
            self.gps_callback, 10)

        # Cameras (use SensorDataQoS to match ZED publishers)
        for name, topic_param in [
            ('front', 'camera_front_topic'),
            ('left', 'camera_left_topic'),
            ('right', 'camera_right_topic')
        ]:
            topic = self.get_parameter(topic_param).value
            setattr(self, f'camera_{name}_sub', self.create_subscription(
                Image, topic,
                lambda msg, n=name: self.camera_callback(msg, n),
                image_qos))
            self.get_logger().info(f'Subscribed to {name} camera: {topic}')

    def create_services(self):
        """Create recording control services"""
        self.start_srv = self.create_service(
            Trigger, 'vill_slam_recorder/start',
            self.start_recording_callback)

        self.stop_srv = self.create_service(
            Trigger, 'vill_slam_recorder/stop',
            self.stop_recording_callback)

        self.toggle_srv = self.create_service(
            SetBool, 'vill_slam_recorder/toggle',
            self.toggle_recording_callback)

    # ==================== Callbacks ====================

    def wheel_odom_callback(self, msg: Odometry):
        with self.data_lock:
            self.current_data.wheel_x = msg.pose.pose.position.x
            self.current_data.wheel_y = msg.pose.pose.position.y
            self.current_data.linear_vel = msg.twist.twist.linear.x
            self.current_data.angular_vel = msg.twist.twist.angular.z

    def slam_odom_callback(self, msg: Odometry):
        if 'slam_odom' not in self._first_callback_logged:
            self._first_callback_logged.add('slam_odom')
            self.get_logger().info(
                f'[slam_odom] first message received: '
                f'x={msg.pose.pose.position.x:.3f} '
                f'y={msg.pose.pose.position.y:.3f} '
                f'z={msg.pose.pose.position.z:.3f} '
                f'frame={msg.header.frame_id}')

        with self.data_lock:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            current_z = msg.pose.pose.position.z

            # Calculate distance traveled
            if self.prev_slam_position is not None:
                dx = current_x - self.prev_slam_position[0]
                dy = current_y - self.prev_slam_position[1]
                dz = current_z - self.prev_slam_position[2]
                self.current_data.slam_distance += np.sqrt(dx**2 + dy**2 + dz**2)

            self.prev_slam_position = (current_x, current_y, current_z)

            self.current_data.slam_x = current_x
            self.current_data.slam_y = current_y
            self.current_data.slam_z = current_z

            # Convert quaternion to Euler
            q = msg.pose.pose.orientation
            # Roll, Pitch, Yaw from quaternion
            sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (q.w * q.y - q.z * q.x)
            pitch = np.arcsin(np.clip(sinp, -1, 1))

            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            self.current_data.slam_roll = np.degrees(roll)
            self.current_data.slam_pitch = np.degrees(pitch)
            self.current_data.slam_yaw = np.degrees(yaw)
            if self.current_data.slam_yaw < 0:
                self.current_data.slam_yaw += 360.0

        # SLAM metrics에도 corrected pose 저장
        new_corr = np.array([current_x, current_y, current_z])
        with self.metrics_lock:
            # corrected pose 프레임 간 step
            if self.corr_pose is not None:
                self.current_metrics.corr_step = float(
                    np.linalg.norm(new_corr - self.corr_pose))

            self.corr_pose = new_corr
            self.current_metrics.corr_x = float(current_x)
            self.current_metrics.corr_y = float(current_y)
            self.current_metrics.corr_z = float(current_z)
            self.current_metrics.corr_yaw_deg = float(self.current_data.slam_yaw)
            # raw vs corrected 차이 (loop closure 보정 누적량)
            if self.raw_pose is not None:
                self.current_metrics.correction_norm = float(
                    np.linalg.norm(self.corr_pose - self.raw_pose))

    def slam_status_callback(self, msg: VillSlamStatus):
        with self.data_lock:
            self.current_data.environment_mode = msg.environment_mode
            self.current_data.pipe_radius = msg.estimated_pipe_radius

        with self.metrics_lock:
            self.current_metrics.num_keyframes = int(msg.num_keyframes)
            self.current_metrics.num_constraints = int(msg.num_constraints)
            self.current_metrics.loop_closure_count = int(msg.loop_closure_count)
            self.current_metrics.estimated_drift = float(msg.estimated_drift)
            self.current_metrics.total_distance = float(msg.total_distance)
            self.current_metrics.environment_mode = int(msg.environment_mode)
            self.current_metrics.pipe_radius = float(msg.estimated_pipe_radius)
            self.current_metrics.pipe_radius_confidence = float(msg.pipe_radius_confidence)
            self.current_metrics.sensors_active = int(msg.sensors_active)
            self.current_metrics.sensors_healthy = int(msg.sensors_healthy)

    def raw_odom_callback(self, msg: Odometry):
        """COIN-LIO raw odometry — 보정 전 포즈 + 발산 감지 지표"""
        self.hz_raw_odom.tick()

        if 'raw_odom' not in self._first_callback_logged:
            self._first_callback_logged.add('raw_odom')
            self.get_logger().info(
                f'[raw_odom] first message: '
                f'x={msg.pose.pose.position.x:.3f} '
                f'frame={msg.header.frame_id}')

        new_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z])

        with self.metrics_lock:
            # 프레임 간 이동량 (발산 감지)
            if self.raw_pose is not None:
                step = float(np.linalg.norm(new_pose - self.raw_pose))
                self.current_metrics.raw_step = step
                # 비정상 점프: 0.5m/frame 이상이면 의심 (10Hz 가정 → 5m/s)
                self.current_metrics.raw_jump = step if step > 0.5 else 0.0

            self.raw_pose = new_pose
            self.current_metrics.raw_x = float(new_pose[0])
            self.current_metrics.raw_y = float(new_pose[1])
            self.current_metrics.raw_z = float(new_pose[2])
            self.current_metrics.vel_linear = float(msg.twist.twist.linear.x)
            self.current_metrics.vel_angular = float(msg.twist.twist.angular.z)

            # Pose covariance (불확실성 - 발산 직전 폭증)
            # 6x6 covariance matrix flat: [xx, xy, xz, xrx, xry, xrz, yx, yy, ...]
            cov = msg.pose.covariance
            # diagonal indices: 0, 7, 14 (xx, yy, zz), 21, 28, 35 (rr, pp, yy)
            self.current_metrics.pose_cov_xyz = float(cov[0] + cov[7] + cov[14])
            self.current_metrics.pose_cov_rot = float(cov[21] + cov[28] + cov[35])

    def _lidar_hz_callback(self, msg: PointCloud2):
        self.hz_lidar.tick()
        # 포인트 수 추적 (특징점 부족 → 발산 신호)
        with self.metrics_lock:
            self.current_metrics.lidar_point_count = int(msg.width * msg.height)

    def _imu_hz_callback(self, msg: Imu):
        self.hz_imu.tick()
        # IMU 가속도/각속도 저장 (발산은 IMU 이상 데이터에서 시작되는 경우 많음)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        with self.metrics_lock:
            self.current_metrics.imu_acc_x = float(ax)
            self.current_metrics.imu_acc_y = float(ay)
            self.current_metrics.imu_acc_z = float(az)
            self.current_metrics.imu_acc_norm = float(
                (ax * ax + ay * ay + az * az) ** 0.5)
            self.current_metrics.imu_gyro_x = float(gx)
            self.current_metrics.imu_gyro_y = float(gy)
            self.current_metrics.imu_gyro_z = float(gz)
            self.current_metrics.imu_gyro_norm = float(
                (gx * gx + gy * gy + gz * gz) ** 0.5)

    def surface_callback(self, msg: SurfaceSection):
        with self.data_lock:
            if msg.is_valid:
                self.current_data.pipe_radius = msg.radius
                self.current_data.pipe_eccentricity = msg.eccentricity

    def temp_humi_callback(self, msg: PointStamped):
        with self.data_lock:
            self.current_data.temperature = msg.point.x
            self.current_data.humidity = msg.point.y

    def gps_callback(self, msg: NavSatFix):
        with self.data_lock:
            self.current_data.latitude = msg.latitude
            self.current_data.longitude = msg.longitude
            self.current_data.altitude = msg.altitude

    def camera_callback(self, msg: Image, camera_name: str):
        # Hz tracking for front camera
        if camera_name == 'front':
            self.hz_cam_front.tick()

        # Diagnostic: log first callback for each camera
        cb_key = f'cam_{camera_name}'
        if cb_key not in self._first_callback_logged:
            self._first_callback_logged.add(cb_key)
            self.get_logger().info(
                f'[{camera_name}] first frame received: '
                f'{msg.width}x{msg.height} encoding={msg.encoding}')

        if not self.is_recording:
            return

        try:
            cv_image = self._image_to_bgr(msg)
            if cv_image is None:
                return
        except Exception as e:
            self.get_logger().error(f'CV bridge error ({camera_name}): {e}',
                                    throttle_duration_sec=5.0)
            return

        current_time = self.get_clock().now().nanoseconds / 1e9

        # Video recording
        if self.get_parameter('enable_video').value:
            self.write_video_frame(camera_name, cv_image, current_time)

        # Image snapshots
        if self.get_parameter('enable_images').value:
            self.save_image_snapshot(camera_name, cv_image)

    def _image_to_bgr(self, msg: Image):
        """Convert ROS Image to BGR numpy WITHOUT cv_bridge.

        cv_bridge is incompatible with NumPy 2.x (segfault in cvtColor2).
        Manual numpy conversion is identical to what zenoh_client.py uses.
        """
        enc = msg.encoding.lower()
        h, w = msg.height, msg.width
        try:
            if enc == 'bgr8':
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            if enc == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if enc == 'bgra8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
                return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            if enc == 'rgba8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
                return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            if enc in ('mono8', '8uc1'):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(
                f'Image conversion failed ({enc}): {e}',
                throttle_duration_sec=5.0)
            return None

        self.get_logger().warn(
            f'Unsupported image encoding: {msg.encoding}',
            throttle_duration_sec=10.0)
        return None

    # ==================== Recording Control ====================

    def start_recording(self) -> bool:
        """Start recording all data"""
        if self.is_recording:
            return False

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Initialize CSV
        if self.get_parameter('enable_csv').value:
            csv_filename = os.path.join(self.csv_path, f'robot_data_{timestamp}.csv')
            self.csv_file = open(csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # Write header
            headers = [
                'timestamp', 'wheel_x', 'wheel_y', 'linear_vel', 'angular_vel',
                'slam_x', 'slam_y', 'slam_z', 'slam_roll', 'slam_pitch', 'slam_yaw',
                'slam_distance', 'temperature', 'humidity',
                'latitude', 'longitude', 'altitude',
                'pipe_radius', 'pipe_eccentricity', 'environment_mode'
            ]
            self.csv_writer.writerow(headers)

        # Initialize SLAM metrics CSV (10Hz, 발산 분석용)
        if self.get_parameter('enable_metrics').value:
            metrics_filename = os.path.join(
                self.csv_path, f'slam_metrics_{timestamp}.csv')
            self.metrics_file = open(metrics_filename, 'w', newline='')
            self.metrics_writer = csv.writer(self.metrics_file)
            metrics_headers = [
                'timestamp',
                # Raw odometry (COIN-LIO)
                'raw_x', 'raw_y', 'raw_z',
                # Corrected pose (VILL-SLAM)
                'corr_x', 'corr_y', 'corr_z', 'corr_yaw_deg',
                # Loop closure correction magnitude
                'correction_norm',
                # Twist
                'vel_linear', 'vel_angular',
                # 발산 감지 핵심
                'raw_step', 'corr_step', 'raw_jump',
                'pose_cov_xyz', 'pose_cov_rot',
                # IMU (발산 직전 이상 감지)
                'imu_acc_x', 'imu_acc_y', 'imu_acc_z', 'imu_acc_norm',
                'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z', 'imu_gyro_norm',
                # LiDAR 환경 인식
                'lidar_point_count',
                # SLAM stats
                'num_keyframes', 'num_constraints', 'loop_closure_count',
                'estimated_drift', 'total_distance',
                'environment_mode', 'pipe_radius', 'pipe_radius_confidence',
                'sensors_active', 'sensors_healthy',
                # Topic frequencies (센서 누락 감지)
                'lidar_hz', 'imu_hz', 'cam_front_hz', 'odom_hz',
                # System resources
                'cpu_pct', 'mem_pct',
            ]
            self.metrics_writer.writerow(metrics_headers)
            self.get_logger().info(f'SLAM metrics CSV: {metrics_filename}')

            # 루프 클로저 이벤트 로그 (발생 시점만 기록)
            loop_filename = os.path.join(
                self.csv_path, f'loop_closures_{timestamp}.csv')
            self.loop_file = open(loop_filename, 'w', newline='')
            self.loop_writer = csv.writer(self.loop_file)
            self.loop_writer.writerow([
                'timestamp', 'loop_count', 'num_keyframes',
                'corr_x', 'corr_y', 'corr_z',
                'raw_x', 'raw_y', 'raw_z',
                'correction_norm', 'estimated_drift', 'total_distance',
            ])
            self.last_loop_count = 0
            self.get_logger().info(f'Loop closure events: {loop_filename}')

        # Video writers는 첫 프레임이 들어올 때 lazy initialization
        # (실제 카메라 해상도/포맷에 맞춰 생성)
        self._pending_video_timestamp = timestamp
        self.video_writers.clear()
        self.frame_counts.clear()

        # Reset snapshot distance
        self.last_snapshot_distance = 0.0

        self.is_recording = True
        self.get_logger().info(f'Recording started: {timestamp}')
        return True

    def stop_recording(self) -> bool:
        """Stop recording all data"""
        if not self.is_recording:
            return False

        self.is_recording = False

        # Close CSV
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        # Close metrics CSV
        if self.metrics_file:
            self.metrics_file.close()
            self.metrics_file = None
            self.metrics_writer = None

        # Close loop closure log
        if self.loop_file:
            self.loop_file.close()
            self.loop_file = None
            self.loop_writer = None

        # Close video writers
        for name, writer in self.video_writers.items():
            writer.release()
            self.get_logger().info(f'Video {name}: {self.frame_counts.get(name, 0)} frames')
        self.video_writers.clear()

        self.get_logger().info('Recording stopped')
        return True

    # ==================== Data Writing ====================

    def write_csv_row(self):
        """Write current data to CSV file"""
        if not self.is_recording or not self.csv_writer:
            return

        with self.data_lock:
            data = self.current_data
            row = [
                time.time(),
                data.wheel_x, data.wheel_y, data.linear_vel, data.angular_vel,
                data.slam_x, data.slam_y, data.slam_z,
                data.slam_roll, data.slam_pitch, data.slam_yaw, data.slam_distance,
                data.temperature, data.humidity,
                data.latitude, data.longitude, data.altitude,
                data.pipe_radius, data.pipe_eccentricity, data.environment_mode
            ]

        try:
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().error(f'CSV write error: {e}')

    def write_metrics_row(self):
        """Write SLAM performance metrics row (for divergence analysis)"""
        if not self.is_recording or not self.metrics_writer:
            return

        # Update live measurements
        cpu = -1.0
        mem = -1.0
        if self.process is not None:
            try:
                cpu = self.process.cpu_percent()
                mem = self.process.memory_percent()
            except Exception:
                pass

        loop_event = None  # capture loop closure event for separate log

        with self.metrics_lock:
            m = self.current_metrics
            m.lidar_hz = self.hz_lidar.hz()
            m.imu_hz = self.hz_imu.hz()
            m.cam_front_hz = self.hz_cam_front.hz()
            m.odom_hz = self.hz_raw_odom.hz()
            m.cpu_pct = cpu
            m.mem_pct = mem

            # 루프 클로저 발생 감지 (count 증가 시점)
            if m.loop_closure_count > self.last_loop_count:
                loop_event = (
                    time.time(), m.loop_closure_count, m.num_keyframes,
                    m.corr_x, m.corr_y, m.corr_z,
                    m.raw_x, m.raw_y, m.raw_z,
                    m.correction_norm, m.estimated_drift, m.total_distance,
                )
                self.last_loop_count = m.loop_closure_count

            row = [
                time.time(),
                m.raw_x, m.raw_y, m.raw_z,
                m.corr_x, m.corr_y, m.corr_z, m.corr_yaw_deg,
                m.correction_norm,
                m.vel_linear, m.vel_angular,
                m.raw_step, m.corr_step, m.raw_jump,
                m.pose_cov_xyz, m.pose_cov_rot,
                m.imu_acc_x, m.imu_acc_y, m.imu_acc_z, m.imu_acc_norm,
                m.imu_gyro_x, m.imu_gyro_y, m.imu_gyro_z, m.imu_gyro_norm,
                m.lidar_point_count,
                m.num_keyframes, m.num_constraints, m.loop_closure_count,
                m.estimated_drift, m.total_distance,
                m.environment_mode, m.pipe_radius, m.pipe_radius_confidence,
                m.sensors_active, m.sensors_healthy,
                m.lidar_hz, m.imu_hz, m.cam_front_hz, m.odom_hz,
                m.cpu_pct, m.mem_pct,
            ]

        try:
            self.metrics_writer.writerow(row)
            self.metrics_file.flush()
        except Exception as e:
            self.get_logger().error(f'Metrics CSV write error: {e}',
                                    throttle_duration_sec=5.0)

        # 루프 클로저 발생 시 별도 이벤트 로그 + 콘솔 출력
        if loop_event is not None and self.loop_writer is not None:
            try:
                self.loop_writer.writerow(loop_event)
                self.loop_file.flush()
                self.get_logger().info(
                    f'[LOOP] #{loop_event[1]} '
                    f'@ ({loop_event[3]:.2f}, {loop_event[4]:.2f}, {loop_event[5]:.2f}) '
                    f'correction={loop_event[9]:.3f}m '
                    f'kfs={loop_event[2]}')
            except Exception as e:
                self.get_logger().error(f'Loop event log error: {e}')

    def write_video_frame(self, camera_name: str, frame: np.ndarray, timestamp: float):
        """Write frame to video file (lazy init on first frame)"""
        # Lazy init: 첫 프레임이 들어왔을 때 실제 크기로 writer 생성
        if camera_name not in self.video_writers:
            timestamp_str = getattr(self, '_pending_video_timestamp', None)
            if timestamp_str is None:
                return
            h, w = frame.shape[:2]
            video_filename = os.path.join(
                self.video_path, f'{camera_name}_{timestamp_str}.mp4')
            fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
            writer = cv2.VideoWriter(
                video_filename, fourcc, float(self.video_fps), (w, h))
            if not writer.isOpened():
                self.get_logger().warn(
                    f'Failed to open video writer for {camera_name} '
                    f'({w}x{h}, codec={self.video_codec}). Trying MJPG fallback')
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                video_filename = video_filename.replace('.mp4', '.avi')
                writer = cv2.VideoWriter(
                    video_filename, fourcc, float(self.video_fps), (w, h))
                if not writer.isOpened():
                    self.get_logger().error(
                        f'Video writer failed for {camera_name} - giving up')
                    return
            self.video_writers[camera_name] = writer
            self.frame_counts[camera_name] = 0
            self.get_logger().info(
                f'Video writer opened: {video_filename} ({w}x{h}@{self.video_fps}fps)')

        try:
            self.video_writers[camera_name].write(frame)
            self.frame_counts[camera_name] = self.frame_counts.get(camera_name, 0) + 1
        except Exception as e:
            self.get_logger().error(
                f'Video write error ({camera_name}): {e}',
                throttle_duration_sec=5.0)

    def save_image_snapshot(self, camera_name: str, frame: np.ndarray):
        """Save image snapshot at distance intervals (with time-based fallback)"""
        with self.data_lock:
            current_distance = self.current_data.slam_distance

        now = time.time()
        save = False
        tag = ''

        # Distance-based trigger (preferred)
        if current_distance - self.last_snapshot_distance >= self.snapshot_interval:
            save = True
            tag = f'{current_distance:.2f}m'
            self.last_snapshot_distance = current_distance

        # Time-based fallback: save every N seconds when distance is stuck
        elif self.snapshot_time_interval > 0:
            last_t = self.last_snapshot_time.get(camera_name, 0.0)
            if now - last_t >= self.snapshot_time_interval:
                save = True
                tag = f't{int(now)}'

        if not save:
            return

        self.last_snapshot_time[camera_name] = now

        filename = os.path.join(
            self.image_path,
            f'{camera_name}_{tag}.jpg')

        try:
            cv2.imwrite(filename, frame)
            self.get_logger().debug(f'Snapshot saved: {filename}')
        except Exception as e:
            self.get_logger().error(f'Image save error: {e}',
                                    throttle_duration_sec=5.0)

    # ==================== Service Callbacks ====================

    def start_recording_callback(self, request, response):
        success = self.start_recording()
        response.success = success
        response.message = 'Recording started' if success else 'Already recording'
        return response

    def stop_recording_callback(self, request, response):
        success = self.stop_recording()
        response.success = success
        response.message = 'Recording stopped' if success else 'Not recording'
        return response

    def toggle_recording_callback(self, request, response):
        if request.data:
            success = self.start_recording()
            response.success = success
            response.message = 'Recording started' if success else 'Already recording'
        else:
            success = self.stop_recording()
            response.success = success
            response.message = 'Recording stopped' if success else 'Not recording'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MultiFormatRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_recording()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        # rclpy may already be shut down by signal handler — ignore errors
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
