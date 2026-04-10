# VILL-SLAM 외부 모니터링 UI 개발 가이드

**작성일: 2026-04-10**

이 문서는 외부 PC에서 VILL-SLAM 로봇의 모니터링 UI를 개발하기 위한 참고 자료입니다.
로봇(Jetson)과 외부 PC 사이의 통신은 **Zenoh**를 사용합니다.

---

## 1. 시스템 아키텍처

```
┌─────────────────────────────┐          ┌─────────────────────────────┐
│  로봇 (Jetson AGX Orin)      │          │  외부 PC (모니터링 UI)        │
│                             │          │                             │
│  Ouster OS1-128 ─┐         │          │  ┌───────────────────────┐  │
│  ZED X Front ────┤ ROS2    │  Zenoh   │  │  PyQt5 UI App         │  │
│  ZED X One 4K ───┤ Nodes   │ TCP:7447 │  │                       │  │
│                  │         │◄────────►│  │  zenoh-python          │  │
│  COIN-LIO SLAM ──┤         │          │  │  + msgpack             │  │
│                  ▼         │          │  │  + pyqtgraph (3D)     │  │
│           zenoh_client     │          │  │  + OpenCV (이미지)     │  │
│           (ROS2 Node)      │          │  └───────────────────────┘  │
└─────────────────────────────┘          └─────────────────────────────┘
```

- 로봇 측: `zenoh_client` ROS2 노드가 ROS2 토픽을 구독하여 Zenoh로 브릿지
- UI 측: 순수 Python 앱 (ROS2 설치 불필요), zenoh-python으로 직접 수신
- 네트워크: 무선랜(현재) / 유선랜(추후), TCP unicast

---

## 2. 로봇 측 하드웨어/소프트웨어 정보

### 2.1 플랫폼
| 항목 | 값 |
|------|-----|
| 보드 | NVIDIA Jetson AGX Orin Developer Kit |
| OS | Ubuntu 22.04, L4T 36.4.4 (JetPack 6.2.1) |
| ROS | ROS2 Humble |
| GPU | Orin (30.7GB VRAM, CUDA 12.6) |

### 2.2 센서 구성
| 센서 | 모델 | ROS2 토픽 prefix |
|------|------|------------------|
| LiDAR | Ouster OS1-128 (128채널, 10Hz) | `/ouster/` |
| 전방 카메라 | ZED X (S/N 40782131) | `/zed_front/` |
| 좌측 카메라 | ZED X One 4K (S/N 319430526) | `/zed_left/` |
| IMU | Ouster 내장 IMU (100Hz) | `/ouster/imu` |

### 2.3 SLAM 알고리즘
- **COIN-LIO** (현재 메인): Fast-LIO2 기반 + Intensity photometric 모듈
- 소스 경로: `/home/test/ros2_ws/src/coin_lio/`
- 설정 파일: `/home/test/ros2_ws/src/coin_lio/config/ouster128.yaml`

---

## 3. Zenoh 통신 프로토콜 정의

### 3.1 연결 설정

| 항목 | 값 |
|------|-----|
| 프로토콜 | Zenoh (TCP) |
| 로봇 리스너 | `tcp/0.0.0.0:7447` |
| UI 접속 주소 | `tcp/<ROBOT_IP>:7447` |
| 모드 | peer (multicast scouting 비활성화) |
| 직렬화 | msgpack (바이너리), JPEG (이미지) |

### 3.2 Zenoh Key-Value 채널 정의

#### Robot → UI (로봇이 발행)

| Zenoh Key | 원본 ROS2 토픽 | 원본 메시지 타입 | 직렬화 | 주기 |
|-----------|---------------|-----------------|--------|------|
| `slam/pose` | `/Odometry` | nav_msgs/Odometry | msgpack | 20Hz |
| `slam/path` | `/path` | nav_msgs/Path | msgpack | 2Hz |
| `slam/pointcloud` | `/vill_slam/dense_map` | sensor_msgs/PointCloud2 (XYZRGB) | msgpack | 1Hz |
| `slam/status` | (내부 생성) | - | msgpack | 5Hz |
| `slam/image/front` | `/zed_front/zed_node/rgb/color/rect/image` | sensor_msgs/Image | JPEG bytes | 5Hz |
| `slam/image/left` | `/zed_left/zed_node/rgb/color/rect/image` | sensor_msgs/Image | JPEG bytes | 3Hz |

#### UI → Robot (UI가 발행)

| Zenoh Key | 대상 ROS2 토픽 | 메시지 타입 | 직렬화 | 설명 |
|-----------|---------------|------------|--------|------|
| `slam/command` | `/slam_command` | std_msgs/String | UTF-8 문자열 | UI 명령 |

### 3.3 데이터 포맷 상세

#### `slam/pose` (msgpack dict)
```python
{
    "timestamp": "2026-04-10 14:22:47.123",   # str, 로봇 시각
    "x": 1.234,          # float, meters
    "y": -0.567,         # float, meters
    "z": 0.089,          # float, meters
    "roll": 0.01,        # float, radians
    "pitch": -0.02,      # float, radians
    "yaw": 1.57,         # float, radians
    "vx": 0.5,           # float, m/s (선속도)
    "wz": 0.1,           # float, rad/s (각속도)
}
```

#### `slam/path` (msgpack dict)
```python
{
    "timestamp": "2026-04-10 14:22:47.123",
    "frame_id": "camera_init",
    "points": [
        [x1, y1, z1],    # [float, float, float] meters
        [x2, y2, z2],
        ...
    ]
}
```

#### `slam/pointcloud` (msgpack dict)
```python
{
    "timestamp": "2026-04-10 14:22:47.123",
    "frame_id": "map",
    "num_points": 5000,
    "points": bytes,      # numpy float32 array, shape (N, 3), tobytes()
    "rgb": bytes,         # numpy uint8 array, shape (N, 3) [R,G,B], tobytes() (optional)
    "intensity": bytes,   # numpy float32 array, shape (N,), tobytes() (optional, raw cloud일 때만)
}
```
- **소스가 `/vill_slam/dense_map`인 경우 (기본)**: `rgb` 필드 포함, `intensity` 없음
  - LiDAR 포인트에 ZED X 전방 카메라 RGB가 입혀진 색상
  - 카메라 FOV 밖의 포인트는 dense_mapper의 fallback (높이 기반 무지개색)
- **소스가 `/cloud_registered`인 경우 (옛 동작)**: `intensity` 필드 포함, `rgb` 없음
- 포인트클라우드는 voxel 다운샘플링 후 전송 (원본 ~128K점 → ~5K점)
- UI에서 복원:
  ```python
  points = np.frombuffer(data["points"], dtype=np.float32).reshape(-1, 3)
  if "rgb" in data:
      rgb = np.frombuffer(data["rgb"], dtype=np.uint8).reshape(-1, 3)  # [R,G,B] 0-255
  ```

#### `slam/status` (msgpack dict)
```python
{
    "timestamp": "2026-04-10 14:22:47.123",
    "slam_state": "running",      # "running" | "idle" | "error"
    "frame_count": 1234,          # int, 처리된 프레임 수
    "num_features": 15000,        # int, 현재 맵 특징점 수
    "cpu_usage": 45.2,            # float, percent
    "mem_usage": 60.1,            # float, percent
    "lidar_hz": 10.0,             # float, LiDAR 수신 주파수
    "imu_hz": 100.0,              # float, IMU 수신 주파수
}
```

#### `slam/image/front`, `slam/image/left` (raw bytes)
```python
# JPEG 압축된 바이너리 (quality=50, 대역폭 절약)
# UI에서 복원:
buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
image = cv2.imdecode(buf, cv2.IMREAD_COLOR)
```

#### `slam/command` (UTF-8 문자열)
```python
# 명령어 목록
"E_STOP"           # 긴급 정지
"START_SLAM"       # SLAM 시작
"STOP_SLAM"        # SLAM 중지
"SAVE_MAP"         # 현재 맵 PCD 저장
"RESET"            # SLAM 초기화
"START_RECORD"     # rosbag 기록 시작
"STOP_RECORD"      # rosbag 기록 중지
```

---

## 4. 외부 PC UI 개발 환경 설정

### 4.1 필수 패키지 설치
```bash
# Python 3.10+ 권장
pip install eclipse-zenoh msgpack-python pyqt5 pyqtgraph opencv-python-headless numpy
```

### 4.2 Zenoh 연결 테스트
```python
import zenoh
import msgpack

# 로봇에 연결
config = zenoh.Config()
config.insert_json5("scouting/multicast/enabled", "false")
config.insert_json5("connect/endpoints", '["tcp/<ROBOT_IP>:7447"]')
session = zenoh.open(config)

# pose 수신 테스트
def on_pose(sample):
    data = msgpack.unpackb(sample.payload.to_bytes())
    print(f"Position: x={data['x']:.3f}, y={data['y']:.3f}, z={data['z']:.3f}")

sub = session.declare_subscriber("slam/pose", on_pose)
input("Press Enter to quit...")
session.close()
```

### 4.3 명령 전송 테스트
```python
import zenoh

config = zenoh.Config()
config.insert_json5("scouting/multicast/enabled", "false")
config.insert_json5("connect/endpoints", '["tcp/<ROBOT_IP>:7447"]')
session = zenoh.open(config)

# 명령 전송
session.put("slam/command", "START_SLAM".encode("utf-8"))
session.close()
```

---

## 5. UI 화면 구성

### 5.1 레이아웃

```
┌──────────────────────────────────────────────────────────────┐
│  VILL-SLAM Monitor                  [● Connected] [192.168.x.x] │
├───────────────────────────┬──────────────────────────────────┤
│                           │  전방 카메라 (slam/image/front)    │
│  3D 포인트클라우드 뷰어     │  ┌────────────────────────────┐  │
│  (pyqtgraph GLViewWidget) │  │        640 x 360            │  │
│                           │  └────────────────────────────┘  │
│  - 포인트클라우드 (흰색)    │  좌측 카메라 (slam/image/left)    │
│  - 경로 궤적 (초록색)      │  ┌────────────────────────────┐  │
│  - 현재 위치 (빨간 점)     │  │        640 x 360            │  │
│                           │  └────────────────────────────┘  │
│                           ├──────────────────────────────────┤
│                           │  상태 패널                        │
│                           │  Position : x=1.23 y=-0.45 z=0.08│
│                           │  Yaw      : 89.5°                │
│                           │  Speed    : 0.5 m/s              │
│                           │  SLAM     : Running (Frame #1234)│
│                           │  LiDAR    : 10.0 Hz              │
│                           │  CPU/MEM  : 45% / 60%            │
├───────────────────────────┴──────────────────────────────────┤
│  [E-STOP]  [Start SLAM]  [Stop SLAM]  [Save Map]  [Record]  │
└──────────────────────────────────────────────────────────────┘
```

### 5.2 기능 요구사항

| 기능 | 설명 |
|------|------|
| 3D 뷰 | 포인트클라우드 + 경로 오버레이, 마우스로 회전/줌 |
| 카메라 뷰 | JPEG 디코드하여 실시간 표시 |
| 상태 표시 | pose, SLAM 상태, 센서 주파수 실시간 갱신 |
| 명령 버튼 | E-STOP, Start/Stop SLAM, Save Map, Record |
| 연결 상태 | Zenoh 연결 여부 표시, 자동 재연결 |

---

## 6. UI 측 핵심 코드 패턴

### 6.1 Zenoh 수신 → Qt 시그널 연동
Zenoh 콜백은 별도 스레드에서 실행되므로, Qt 메인 스레드와 시그널로 연동해야 합니다.

```python
from PyQt5.QtCore import QObject, pyqtSignal
import zenoh
import msgpack

class ZenohReceiver(QObject):
    """Zenoh 데이터를 Qt 시그널로 변환"""
    pose_received = pyqtSignal(dict)
    path_received = pyqtSignal(dict)
    pointcloud_received = pyqtSignal(dict)
    status_received = pyqtSignal(dict)
    image_front_received = pyqtSignal(bytes)
    image_left_received = pyqtSignal(bytes)

    def __init__(self, robot_ip: str, port: int = 7447):
        super().__init__()
        self.robot_ip = robot_ip
        self.port = port
        self.session = None
        self.subscribers = []

    def connect(self):
        config = zenoh.Config()
        config.insert_json5("scouting/multicast/enabled", "false")
        config.insert_json5("connect/endpoints",
                           f'["tcp/{self.robot_ip}:{self.port}"]')
        self.session = zenoh.open(config)

        # msgpack 채널
        for key, signal in [
            ("slam/pose", self.pose_received),
            ("slam/path", self.path_received),
            ("slam/pointcloud", self.pointcloud_received),
            ("slam/status", self.status_received),
        ]:
            sub = self.session.declare_subscriber(
                key, lambda sample, sig=signal: sig.emit(
                    msgpack.unpackb(sample.payload.to_bytes())
                )
            )
            self.subscribers.append(sub)

        # 이미지 채널 (raw JPEG bytes)
        for key, signal in [
            ("slam/image/front", self.image_front_received),
            ("slam/image/left", self.image_left_received),
        ]:
            sub = self.session.declare_subscriber(
                key, lambda sample, sig=signal: sig.emit(
                    sample.payload.to_bytes()
                )
            )
            self.subscribers.append(sub)

    def send_command(self, command: str):
        if self.session:
            self.session.put("slam/command", command.encode("utf-8"))

    def disconnect(self):
        if self.session:
            self.session.close()
            self.session = None
```

### 6.2 포인트클라우드 복원 및 렌더링
```python
import numpy as np
import pyqtgraph.opengl as gl

def update_pointcloud(self, data: dict):
    points = np.frombuffer(data["points"], dtype=np.float32).reshape(-1, 3)

    # 우선순위 1: RGB (dense_map의 카메라 색상)
    if "rgb" in data and data["rgb"]:
        rgb = np.frombuffer(data["rgb"], dtype=np.uint8).reshape(-1, 3)
        # pyqtgraph는 RGBA float [0..1] 형식을 기대
        colors = np.ones((len(points), 4), dtype=np.float32)
        colors[:, :3] = rgb.astype(np.float32) / 255.0
    # 우선순위 2: intensity (raw cloud 컬러맵)
    elif "intensity" in data and data["intensity"]:
        intensity = np.frombuffer(data["intensity"], dtype=np.float32)
        colors = intensity_to_colormap(intensity)  # (N, 4) RGBA
    # 폴백: 단색 회색
    else:
        colors = np.ones((len(points), 4), dtype=np.float32)
        colors[:, :3] = 0.8

    self.scatter.setData(pos=points, color=colors, size=2)
```

### 6.3 이미지 표시
```python
import cv2
import numpy as np
from PyQt5.QtGui import QImage, QPixmap

def update_camera_image(self, jpeg_bytes: bytes):
    buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    h, w, ch = img_rgb.shape
    qimg = QImage(img_rgb.data, w, h, ch * w, QImage.Format_RGB888)
    self.camera_label.setPixmap(QPixmap.fromImage(qimg).scaled(640, 360))
```

---

## 7. 로봇 측 실행 방법 (참고)

```bash
# 1. Ouster IP 설정 (재부팅 후)
sudo ip addr add 169.254.241.1/24 dev eno1

# 2. 센서 드라이버 실행
source /home/test/ros2_ws/install/setup.bash
ros2 launch vill_slam sensors.launch.py

# 3. COIN-LIO SLAM 실행
ros2 launch coin_lio mapping.launch.py

# 4. Zenoh Client 실행 (구현 후)
ros2 launch vill_slam zenoh_client.launch.py
```

---

## 8. 좌표계 참고

| Frame | 설명 |
|-------|------|
| `camera_init` | COIN-LIO SLAM 월드 프레임 (SLAM 시작 위치가 원점) |
| `body` | 로봇 본체 프레임 |
| `os_sensor` | Ouster LiDAR 프레임 |
| `zed_front_left_camera_frame` | ZED X 전방 카메라 프레임 |

- SLAM 출력(`/Odometry`, `/path`, `/cloud_registered`)의 frame_id = `camera_init`
- 포인트클라우드와 경로는 모두 `camera_init` 프레임 기준

---

## 9. 대역폭 예상치

| 채널 | 원본 크기 | 전송 크기 (압축 후) | 주기 | 대역폭 |
|------|----------|-------------------|------|--------|
| slam/pose | - | ~100 B | 20Hz | ~2 KB/s |
| slam/path | - | ~10 KB (1000점) | 2Hz | ~20 KB/s |
| slam/pointcloud | ~1.5 MB | ~60 KB (5000점) | 1Hz | ~60 KB/s |
| slam/status | - | ~200 B | 5Hz | ~1 KB/s |
| slam/image/front | ~2 MB | ~30 KB (JPEG q50) | 5Hz | ~150 KB/s |
| slam/image/left | ~2 MB | ~30 KB (JPEG q50) | 3Hz | ~90 KB/s |
| **합계** | | | | **~320 KB/s (~2.5 Mbps)** |

WiFi 환경에서 충분히 처리 가능한 대역폭입니다.

---

## 10. 개발 우선순위

1. **zenoh 연결 + pose 수신** (최소 동작 확인)
2. **상태 패널 표시** (pose + status)
3. **카메라 이미지 표시**
4. **3D 포인트클라우드 뷰**
5. **경로 오버레이**
6. **명령 버튼 (E-STOP 등)**
7. **연결 상태 관리 + 자동 재연결**
