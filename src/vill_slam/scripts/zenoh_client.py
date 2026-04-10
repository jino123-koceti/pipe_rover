#!/usr/bin/env python3
"""
Zenoh Client Node
로봇(Jetson) ROS2 토픽 → Zenoh → 외부 PC UI 브릿지

Zenoh 발행 (Robot → UI):
  - "slam/pose"          ← /Odometry (nav_msgs/Odometry → msgpack)
  - "slam/path"          ← /path (nav_msgs/Path → msgpack)
  - "slam/pointcloud"    ← /cloud_registered (PointCloud2 → voxel downsample → msgpack)
  - "slam/status"        ← 내부 생성 (시스템 상태 → msgpack)
  - "slam/image/front"   ← /zed_front/zed_node/rgb/color/rect/image (Image → JPEG bytes)
  - "slam/image/left"    ← /zed_left/zed_node/rgb/color/rect/image (Image → JPEG bytes)

Zenoh 구독 (UI → Robot):
  - "slam/command"       → /slam_command (String)
"""

import math
from datetime import datetime

import numpy as np
import cv2
import msgpack

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, Image

try:
    import zenoh
except ImportError:
    raise ImportError(
        "zenoh-python이 설치되지 않았습니다. "
        "pip install eclipse-zenoh 으로 설치하세요."
    )


class ZenohClient(Node):
    """ROS2 ↔ Zenoh 브릿지 노드"""

    def __init__(self):
        super().__init__('zenoh_client')
        self._declare_parameters()
        self._load_parameters()

        # Zenoh 세션
        self.session = self._init_zenoh_session()
        if not self.session:
            return

        # Zenoh 구독 (UI → Robot)
        self._init_zenoh_subscribers()

        # ROS2 발행자 (Zenoh → ROS2)
        self.cmd_pub = self.create_publisher(String, '/slam_command', 10)

        # 서비스 클라이언트 (UI 명령 → 실제 서비스 호출)
        self.record_start_client = self.create_client(
            Trigger, '/vill_slam_recorder/start')
        self.record_stop_client = self.create_client(
            Trigger, '/vill_slam_recorder/stop')

        # ROS2 구독자 (ROS2 → Zenoh)
        self._init_ros_subscribers()

        # 상태 데이터 저장
        self.latest_odom = None
        self.slam_frame_count = 0
        self.lidar_hz_counter = _HzCounter()
        self.imu_hz_counter = _HzCounter()

        # 주기적 상태 발행 타이머
        self.status_timer = self.create_timer(
            1.0 / self.status_rate, self._publish_status
        )

        self.get_logger().info("Zenoh Client 초기화 완료")
        self.get_logger().info(f"  Zenoh: tcp/0.0.0.0:{self.zenoh_port} (peer)")
        self.get_logger().info(f"  Pose: {self.pose_rate}Hz, Path: {self.path_rate}Hz, "
                               f"PointCloud: {self.pc_rate}Hz")
        self.get_logger().info(f"  Image Front: {self.img_front_rate}Hz, "
                               f"Image Left: {self.img_left_rate}Hz")

    # ------------------------------------------------------------------ #
    #  파라미터
    # ------------------------------------------------------------------ #
    def _declare_parameters(self):
        # Zenoh
        self.declare_parameter('zenoh_port', 7447)
        self.declare_parameter('zenoh_mode', 'peer')
        self.declare_parameter('zenoh_router', '')

        # Zenoh keys
        self.declare_parameter('pose_key', 'slam/pose')
        self.declare_parameter('path_key', 'slam/path')
        self.declare_parameter('pointcloud_key', 'slam/pointcloud')
        self.declare_parameter('status_key', 'slam/status')
        self.declare_parameter('image_front_key', 'slam/image/front')
        self.declare_parameter('image_left_key', 'slam/image/left')
        self.declare_parameter('command_key', 'slam/command')

        # ROS2 토픽
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('path_topic', '/path')
        # /vill_slam/dense_map: Dense RGB-D 매핑 결과 (LiDAR 포인트 + ZED RGB 색상)
        # /cloud_registered: COIN-LIO raw 포인트클라우드 (intensity만)
        self.declare_parameter('pointcloud_topic', '/vill_slam/dense_map')
        self.declare_parameter('image_front_topic',
                               '/zed_front/zed_node/rgb/color/rect/image')
        self.declare_parameter('image_left_topic',
                               '/zed_left/zed_node/rgb/color/rect/image')

        # 발행 주기 (Hz)
        self.declare_parameter('pose_rate', 20.0)
        self.declare_parameter('path_rate', 2.0)
        self.declare_parameter('pointcloud_rate', 1.0)
        self.declare_parameter('status_rate', 5.0)
        self.declare_parameter('image_front_rate', 5.0)
        self.declare_parameter('image_left_rate', 3.0)

        # 포인트클라우드 다운샘플링
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('max_points', 5000)

        # 이미지 JPEG 품질 (0-100)
        self.declare_parameter('jpeg_quality', 50)

    def _load_parameters(self):
        self.zenoh_port = self.get_parameter('zenoh_port').value
        self.zenoh_mode = self.get_parameter('zenoh_mode').value
        self.zenoh_router = self.get_parameter('zenoh_router').value

        self.pose_key = self.get_parameter('pose_key').value
        self.path_key = self.get_parameter('path_key').value
        self.pc_key = self.get_parameter('pointcloud_key').value
        self.status_key = self.get_parameter('status_key').value
        self.img_front_key = self.get_parameter('image_front_key').value
        self.img_left_key = self.get_parameter('image_left_key').value
        self.command_key = self.get_parameter('command_key').value

        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.pc_topic = self.get_parameter('pointcloud_topic').value
        self.img_front_topic = self.get_parameter('image_front_topic').value
        self.img_left_topic = self.get_parameter('image_left_topic').value

        self.pose_rate = self.get_parameter('pose_rate').value
        self.path_rate = self.get_parameter('path_rate').value
        self.pc_rate = self.get_parameter('pointcloud_rate').value
        self.status_rate = self.get_parameter('status_rate').value
        self.img_front_rate = self.get_parameter('image_front_rate').value
        self.img_left_rate = self.get_parameter('image_left_rate').value

        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_points = self.get_parameter('max_points').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

    # ------------------------------------------------------------------ #
    #  Zenoh 초기화
    # ------------------------------------------------------------------ #
    def _init_zenoh_session(self):
        try:
            config = zenoh.Config()
            # DDS 포트 충돌 방지: multicast scouting 비활성화
            config.insert_json5("scouting/multicast/enabled", "false")
            # TCP 리스너 (외부 PC 접속용)
            config.insert_json5(
                "listen/endpoints",
                f'["tcp/0.0.0.0:{self.zenoh_port}"]'
            )

            if self.zenoh_mode == 'client' and self.zenoh_router:
                config.insert_json5(
                    "connect/endpoints",
                    f'["tcp/{self.zenoh_router}:{self.zenoh_port}"]'
                )
                session = zenoh.open(config)
                self.get_logger().info(
                    f"Zenoh 세션 열림 (Client: {self.zenoh_router}:{self.zenoh_port})"
                )
            else:
                session = zenoh.open(config)
                self.get_logger().info(
                    f"Zenoh 세션 열림 (Peer, TCP: 0.0.0.0:{self.zenoh_port})"
                )
            return session

        except Exception as e:
            self.get_logger().error(f"Zenoh 세션 열기 실패: {e}")
            return None

    def _init_zenoh_subscribers(self):
        """Zenoh 구독: UI 명령 수신"""
        self.zenoh_cmd_sub = self.session.declare_subscriber(
            self.command_key, self._on_zenoh_command
        )
        self.get_logger().info(f"  Zenoh 구독: {self.command_key}")

    # ------------------------------------------------------------------ #
    #  ROS2 구독자
    # ------------------------------------------------------------------ #
    def _init_ros_subscribers(self):
        # SLAM 토픽은 기본 QoS
        default_qos = 10

        # 카메라 이미지는 RELIABLE QoS
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 주기 제한용 타임스탬프
        self._last_pose_t = 0.0
        self._last_path_t = 0.0
        self._last_pc_t = 0.0
        self._last_img_front_t = 0.0
        self._last_img_left_t = 0.0

        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, default_qos)
        self.create_subscription(
            Path, self.path_topic, self._on_path, default_qos)
        self.create_subscription(
            PointCloud2, self.pc_topic, self._on_pointcloud, default_qos)
        self.create_subscription(
            Image, self.img_front_topic, self._on_image_front, image_qos)
        self.create_subscription(
            Image, self.img_left_topic, self._on_image_left, image_qos)

    # ------------------------------------------------------------------ #
    #  Zenoh → ROS2 (UI 명령)
    # ------------------------------------------------------------------ #
    def _on_zenoh_command(self, sample):
        try:
            raw = sample.payload
            if hasattr(raw, 'to_bytes'):
                data = raw.to_bytes()
            elif isinstance(raw, (bytes, bytearray, memoryview)):
                data = bytes(raw)
            else:
                data = str(raw).encode('utf-8')

            command = data.decode('utf-8').strip()
            self.get_logger().info(f"명령 수신 (Zenoh→ROS2): {command}")

            # /slam_command 토픽으로도 항상 발행 (다른 구독자용)
            msg = String()
            msg.data = command
            self.cmd_pub.publish(msg)

            # 알려진 명령은 직접 서비스 호출로 디스패치
            if command == 'START_RECORD':
                self._call_recorder(self.record_start_client, 'start')
            elif command == 'STOP_RECORD':
                self._call_recorder(self.record_stop_client, 'stop')

        except Exception as e:
            self.get_logger().error(f"명령 수신 오류: {e}")

    def _call_recorder(self, client, name: str):
        """Recorder 서비스 비동기 호출"""
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    f"Recorder {name} 서비스 사용 불가 — vill_slam_recorder 노드가 실행 중인지 확인하세요")
                return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, n=name: self._on_recorder_response(f, n))

    def _on_recorder_response(self, future, name: str):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Recorder {name} OK: {resp.message}")
            else:
                self.get_logger().warn(f"Recorder {name} 실패: {resp.message}")
        except Exception as e:
            self.get_logger().error(f"Recorder {name} 호출 오류: {e}")

    # ------------------------------------------------------------------ #
    #  ROS2 → Zenoh (데이터 브릿지)
    # ------------------------------------------------------------------ #
    def _on_odom(self, msg: Odometry):
        if not self._rate_ok('_last_pose_t', self.pose_rate):
            return

        self.latest_odom = msg
        self.slam_frame_count += 1

        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        roll, pitch, yaw = _quaternion_to_euler(q.x, q.y, q.z, q.w)

        pose_data = {
            'timestamp': _now_str(),
            'x': p.x,
            'y': p.y,
            'z': p.z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'vx': msg.twist.twist.linear.x,
            'wz': msg.twist.twist.angular.z,
        }

        self._zenoh_put_msgpack(self.pose_key, pose_data)

    def _on_path(self, msg: Path):
        if not self._rate_ok('_last_path_t', self.path_rate):
            return

        points = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            points.append([p.x, p.y, p.z])

        path_data = {
            'timestamp': _now_str(),
            'frame_id': msg.header.frame_id,
            'points': points,
        }

        self._zenoh_put_msgpack(self.path_key, path_data)

    def _on_pointcloud(self, msg: PointCloud2):
        if not self._rate_ok('_last_pc_t', self.pc_rate):
            return
        self.lidar_hz_counter.tick()

        try:
            points, intensity, rgb = _pointcloud2_to_numpy(msg)

            # Voxel 다운샘플링 (intensity와 rgb 모두 동기화)
            points, intensity, rgb = self._voxel_downsample(points, intensity, rgb)

            pc_data = {
                'timestamp': _now_str(),
                'frame_id': msg.header.frame_id,
                'num_points': len(points),
                'points': points.astype(np.float32).tobytes(),
            }
            if intensity is not None:
                pc_data['intensity'] = intensity.astype(np.float32).tobytes()
            if rgb is not None:
                # RGB는 (N, 3) uint8 → bytes (UI에서 reshape(-1, 3))
                pc_data['rgb'] = rgb.astype(np.uint8).tobytes()

            self._zenoh_put_msgpack(self.pc_key, pc_data)

        except Exception as e:
            self.get_logger().error(f"PointCloud 변환 오류: {e}", throttle_duration_sec=5.0)

    def _on_image_front(self, msg: Image):
        if not self._rate_ok('_last_img_front_t', self.img_front_rate):
            return
        jpeg = self._image_to_jpeg(msg)
        if jpeg is not None:
            self._zenoh_put_raw(self.img_front_key, jpeg)

    def _on_image_left(self, msg: Image):
        if not self._rate_ok('_last_img_left_t', self.img_left_rate):
            return
        jpeg = self._image_to_jpeg(msg)
        if jpeg is not None:
            self._zenoh_put_raw(self.img_left_key, jpeg)

    # ------------------------------------------------------------------ #
    #  주기적 상태 발행
    # ------------------------------------------------------------------ #
    def _publish_status(self):
        if not self.session:
            return

        try:
            import psutil
            cpu = psutil.cpu_percent()
            mem = psutil.virtual_memory().percent
        except ImportError:
            cpu = -1.0
            mem = -1.0

        status = {
            'timestamp': _now_str(),
            'slam_state': 'running' if self.latest_odom else 'idle',
            'frame_count': self.slam_frame_count,
            'cpu_usage': cpu,
            'mem_usage': mem,
            'lidar_hz': round(self.lidar_hz_counter.hz(), 1),
        }

        self._zenoh_put_msgpack(self.status_key, status)

    # ------------------------------------------------------------------ #
    #  유틸리티
    # ------------------------------------------------------------------ #
    def _rate_ok(self, attr: str, target_hz: float) -> bool:
        """주기 제한: target_hz 이하로만 콜백 통과"""
        now = self.get_clock().now().nanoseconds / 1e9
        last = getattr(self, attr, 0.0)
        if target_hz <= 0 or (now - last) < (1.0 / target_hz):
            return False
        setattr(self, attr, now)
        return True

    def _zenoh_put_msgpack(self, key: str, data: dict):
        """msgpack 직렬화 후 Zenoh put"""
        if not self.session:
            return
        try:
            packed = msgpack.packb(data, use_bin_type=True)
            self.session.put(key, packed)
        except Exception as e:
            self.get_logger().error(f"Zenoh put 오류 ({key}): {e}",
                                   throttle_duration_sec=5.0)

    def _zenoh_put_raw(self, key: str, data: bytes):
        """raw bytes를 Zenoh put"""
        if not self.session:
            return
        try:
            self.session.put(key, data)
        except Exception as e:
            self.get_logger().error(f"Zenoh put 오류 ({key}): {e}",
                                   throttle_duration_sec=5.0)

    def _voxel_downsample(self, points: np.ndarray,
                          intensity: np.ndarray | None,
                          rgb: np.ndarray | None = None):
        """간이 voxel grid 다운샘플링 (intensity, rgb 동기화)"""
        if len(points) <= self.max_points:
            return points, intensity, rgb

        vs = self.voxel_size
        # 복셀 인덱스 계산
        voxel_idx = np.floor(points / vs).astype(np.int32)
        # 유니크 복셀만 남기기
        _, unique_indices = np.unique(
            voxel_idx, axis=0, return_index=True
        )

        # max_points 초과 시 균일 샘플링
        if len(unique_indices) > self.max_points:
            step = len(unique_indices) // self.max_points
            unique_indices = np.sort(unique_indices)[::step][:self.max_points]

        points = points[unique_indices]
        if intensity is not None:
            intensity = intensity[unique_indices]
        if rgb is not None:
            rgb = rgb[unique_indices]

        return points, intensity, rgb

    def _image_to_jpeg(self, msg: Image) -> bytes | None:
        """ROS2 Image → JPEG bytes 변환"""
        try:
            # encoding에 따라 numpy 변환
            if msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgra8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 4)
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            elif msg.encoding == 'rgba8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 4)
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
            elif msg.encoding in ('mono8', '8UC1'):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width)
            else:
                self.get_logger().warn(
                    f"지원하지 않는 이미지 encoding: {msg.encoding}",
                    throttle_duration_sec=10.0)
                return None

            # 해상도 축소 (가로 640px 이하로)
            if img.shape[1] > 640:
                scale = 640.0 / img.shape[1]
                new_w = 640
                new_h = int(img.shape[0] * scale)
                img = cv2.resize(img, (new_w, new_h),
                                 interpolation=cv2.INTER_AREA)

            _, buf = cv2.imencode(
                '.jpg', img,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            return buf.tobytes()

        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}",
                                   throttle_duration_sec=5.0)
            return None

    def destroy_node(self):
        if self.session:
            self.session.close()
            self.get_logger().info("Zenoh 세션 닫힘")
        super().destroy_node()


# ====================================================================== #
#  헬퍼 함수 (모듈 레벨)
# ====================================================================== #

def _now_str() -> str:
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def _quaternion_to_euler(x, y, z, w):
    """Quaternion → (roll, pitch, yaw) in radians"""
    # roll (x-axis)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return roll, pitch, yaw


def _pointcloud2_to_numpy(msg: PointCloud2):
    """
    PointCloud2 → (points: Nx3 float32, intensity: N float32 or None,
                   rgb: Nx3 uint8 or None)

    XYZI/XYZRGB 필드 오프셋을 직접 파싱합니다 (sensor_msgs_py 없이도 동작).
    """
    # 필드 이름 → 오프셋 매핑
    field_map = {}
    for f in msg.fields:
        field_map[f.name] = (f.offset, f.datatype)

    point_step = msg.point_step
    n_points = msg.width * msg.height
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32), None, None

    # XYZ 추출
    x_off = field_map['x'][0]
    y_off = field_map['y'][0]
    z_off = field_map['z'][0]

    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)
    points = np.zeros((n_points, 3), dtype=np.float32)
    for i, off in enumerate([x_off, y_off, z_off]):
        points[:, i] = raw[:, off:off+4].view(np.float32).flatten()

    # NaN 제거 마스크 (intensity/rgb에도 적용)
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]

    # Intensity 추출 (있으면)
    intensity = None
    if 'intensity' in field_map:
        i_off = field_map['intensity'][0]
        intensity = raw[:, i_off:i_off+4].view(np.float32).flatten()
        intensity = intensity[valid]

    # RGB 추출 (PCL XYZRGB 포맷: rgb 필드는 4바이트, 보통 BGRA 또는 0RGB packed)
    rgb = None
    if 'rgb' in field_map or 'rgba' in field_map:
        rgb_key = 'rgb' if 'rgb' in field_map else 'rgba'
        rgb_off = field_map[rgb_key][0]
        # 4 bytes를 uint8 4채널로 해석 (PCL은 BGRA 순서로 packing)
        rgb_bytes = raw[:, rgb_off:rgb_off+4]  # (N, 4) uint8
        # PCL packed RGB: byte 0=B, 1=G, 2=R, 3=A (little-endian float32)
        rgb = np.zeros((n_points, 3), dtype=np.uint8)
        rgb[:, 0] = rgb_bytes[:, 2]  # R
        rgb[:, 1] = rgb_bytes[:, 1]  # G
        rgb[:, 2] = rgb_bytes[:, 0]  # B
        rgb = rgb[valid]

    return points, intensity, rgb


class _HzCounter:
    """간단한 주파수 측정기"""
    def __init__(self, window: float = 2.0):
        self._window = window
        self._times: list[float] = []

    def tick(self):
        import time
        now = time.monotonic()
        self._times.append(now)
        # 윈도우 밖 제거
        cutoff = now - self._window
        self._times = [t for t in self._times if t > cutoff]

    def hz(self) -> float:
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0:
            return 0.0
        return (len(self._times) - 1) / span


def main(args=None):
    rclpy.init(args=args)
    node = ZenohClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
