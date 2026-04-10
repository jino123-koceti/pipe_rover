# pipe_rover

파이프/복도 환경 자율주행 점검 로봇용 **VILL-SLAM** (Visual-Inertial-Laser-Lidar SLAM) 시스템.
NVIDIA Jetson AGX Orin + Ouster OS1-128 + ZED X 카메라 기반 ROS 2 Humble 워크스페이스.

## 시스템 구성

| 센서 | 모델 | 용도 |
|------|------|------|
| LiDAR | Ouster OS1-128 (128ch, 10Hz) | 3D 포인트 클라우드 + intensity 이미지 |
| 전방 카메라 | ZED X (GMSL) | RGB 이미지, IMU |
| 좌측 카메라 | ZED X One 4K | RGB 이미지 |
| IMU | Ouster 내장 (100Hz) | 메인 IMU |
| 라인 레이저 | 커스텀 프로파일러 | 파이프 단면 측정 |

- **플랫폼**: NVIDIA Jetson AGX Orin, L4T 36.4.4 (JetPack 6.2.1), Ubuntu 22.04
- **ROS**: ROS 2 Humble
- **SDK**: ZED SDK 5.2.3, Ouster SDK 0.11.1, GTSAM 4.2

## 아키텍처

```
Ouster LiDAR + IMU            ZED X Cameras
       │                           │
       ▼                           ▼
┌─────────────┐              ┌───────────┐
│  COIN-LIO   │              │ Line Laser│
│ (photometric│              │ Processor │
│  LiDAR-IMU) │              └─────┬─────┘
└──────┬──────┘                    │
       │ /Odometry                 │
       ▼                           ▼
┌─────────────────────────────────────────┐
│         VILL-SLAM (vill_slam_node)      │
│  ├─ Keyframe management                 │
│  ├─ GTSAM iSAM2 pose graph              │
│  ├─ ICP loop closure (with cooldown)    │
│  ├─ Cylinder/Plane factor (pipe/hall)   │
│  └─ Corrected odometry + path           │
└──────┬──────────────────────────────────┘
       │ /vill_slam/odometry, /path
       ▼
┌─────────────────┐    ┌──────────────────┐
│  Dense Mapper   │    │   Recorder       │
│  (LiDAR + RGB)  │    │  (CSV + video +  │
│  RGB-colorized  │    │   SLAM metrics)  │
│  point cloud    │    └──────────────────┘
└────────┬────────┘             │
         │                      ▼
         ▼              /home/test/vill_slam_data/
    /vill_slam/dense_map       ├─ robot_data_*.csv
         │                     ├─ slam_metrics_*.csv
         ▼                     ├─ loop_closures_*.csv
 ┌───────────────┐             ├─ video/*.mp4
 │ Zenoh Bridge  │             └─ images/*.jpg
 │  (→ 외부 UI)  │
 └───────────────┘
```

## 패키지 구조

| 패키지 | 설명 |
|--------|------|
| [`coin_lio`](src/coin_lio/) | COIN-LIO (ICRA 2024) ROS 2 포팅. Fast-LIO2 + photometric intensity 모듈. Ouster 메타데이터 기반 투영, FIR 라인 제거, 밝기 필터. |
| [`vill_slam`](src/vill_slam/) | 메인 SLAM 노드. GTSAM iSAM2 포즈 그래프, ICP 루프 클로저, 키프레임 관리, Dense RGB-D 매퍼, Zenoh 클라이언트, 통합 launch. |
| [`vill_slam_msgs`](src/vill_slam_msgs/) | 커스텀 메시지 (`VillSlamStatus`, `SurfaceSection`, `LaserLine`, `GeometryConstraint`). |
| [`vill_slam_recorder`](src/vill_slam_recorder/) | 다중 포맷 데이터 기록기. CSV(10Hz), MP4 영상, 거리/시간 기반 이미지 스냅샷, **SLAM 성능 metrics 40+ 컬럼**, 루프 클로저 이벤트 로그. |
| [`vill_slam_ui`](src/vill_slam_ui/) | PyQt5 모니터링 UI (로봇 측). |
| [`line_laser_driver`](src/line_laser_driver/) | 라인 레이저 GPIO 컨트롤, 카메라 동기화, 프로파일 추출. |

## 주요 기능

### COIN-LIO Photometric 모드
- Fast-LIO2 기반 + Ouster intensity 이미지로 photometric error 최적화
- 파이프/복도처럼 기하학적 degeneracy가 심한 환경에서 축 방향 모션 추정 개선
- Ouster 메타데이터(`beam_altitude_angles`, `pixel_shift_by_row`) 기반 정확한 투영
- FIR 필터로 LiDAR intensity 라인 아티팩트 제거

### 루프 클로저 (GTSAM iSAM2)
- 거리(0.5m)/회전(0.3rad) 기반 키프레임 자동 선택
- 증분 포즈 그래프 최적화
- ICP 스캔 매칭으로 루프 후보 검증 (fitness threshold + Huber robust noise)
- **Cooldown 메커니즘**: 루프 발생 후 N개 키프레임 동안 다음 루프 시도 금지 → false positive 방지
- Cylinder/Plane Factor 통합 (환경 모드별)

### Dense RGB-D 매핑
- LiDAR 포인트 + ZED X RGB 이미지 융합
- Camera-LiDAR extrinsic 기반 포인트 색상화
- PCD 파일 저장 서비스

### SLAM 성능 모니터링 (발산 분석용)
- **3종 CSV 동시 기록**:
  - `robot_data_*.csv` — 휠/SLAM 포즈 + 환경 센서
  - `slam_metrics_*.csv` — raw vs corrected 포즈, 프레임 점프, IMU acc/gyro, pose covariance, 토픽 Hz, CPU/MEM
  - `loop_closures_*.csv` — 루프 이벤트 상세
- 주행 후 발산 시점과 원인 정량 분석 가능

### Zenoh 외부 UI 브리지
- 로봇(Jetson) ROS 2 토픽 → Zenoh → 외부 PC PyQt5 UI
- Pose, Path, PointCloud (XYZRGB), Status, Camera images
- 명령 역방향 전송 (`START_RECORD`, `STOP_RECORD`, ...)

## 빌드

이 repo는 VILL-SLAM 고유 패키지만 포함합니다. 다음 외부 의존성을 별도로 설치해야 합니다:

```bash
# ROS 2 Humble base packages
sudo apt install ros-humble-desktop ros-humble-pcl-ros ros-humble-tf2-ros \
    ros-humble-gtsam ros-humble-cv-bridge ros-humble-image-transport

# External drivers (별도 워크스페이스에서 빌드)
#  - fast_lio, FAST-LIVO2 (reference)
#  - ouster-ros
#  - zed-ros2-wrapper, zed-ros2-interfaces, zed-ros2-description
#  - rpg_vikit (vikit_common, vikit_ros)
#  - line_laser_driver dependencies
```

Workspace 빌드:

```bash
cd ~/ros2_ws
colcon build --packages-select \
    vill_slam_msgs vill_slam vill_slam_recorder vill_slam_ui \
    coin_lio line_laser_driver \
    --symlink-install

source install/setup.bash
```

## 실행

### 통합 실행 (전체 시스템)
```bash
# Ouster 네트워크 설정 (재부팅 후 필요)
sudo ip addr add 169.254.241.1/24 dev eno1

# 통합 launch
ros2 launch vill_slam vill_slam_integrated.launch.py
```

환경 모드 선택:
```bash
ros2 launch vill_slam vill_slam_integrated.launch.py environment:=corridor
ros2 launch vill_slam vill_slam_integrated.launch.py environment:=pipe
```

### 옵션
```bash
# 센서 없이 rosbag 재생
ros2 launch vill_slam vill_slam_integrated.launch.py \
    launch_sensors:=false use_sim_time:=true

# 최소 구성 (UI/레코더 없이)
ros2 launch vill_slam vill_slam_integrated.launch.py \
    use_ui:=false use_recorder:=false
```

### 녹화 제어
```bash
# 녹화 시작/정지 (UI 또는 직접 서비스 호출)
ros2 service call /vill_slam_recorder/start std_srvs/srv/Trigger
ros2 service call /vill_slam_recorder/stop std_srvs/srv/Trigger

# 루프 클로저/SLAM 통계 실시간
ros2 topic echo /vill_slam/status --once
```

저장 경로: `/home/test/vill_slam_data/{csv,video,images}/`

## 검증 결과

**복도 한 바퀴 주행 테스트 (2026-04-10)**

| 항목 | 값 |
|------|-----|
| 주행 시간 | 10.4 분 |
| 주행 거리 | 126.9 m |
| 복귀 오차 | 0.54 m |
| 키프레임 | 127 |
| 루프 클로저 | 5 회 (비율 4%) |
| 루프 클로저 평균 보정 | 0.144 m |
| `raw_step > 0.5m` 점프 | **0 회** |
| 결과 | **발산 없이 완주** |

자세한 분석과 튜닝 이력은 [docs/PROGRESS_REPORT.md](docs/PROGRESS_REPORT.md) 참고.

## 문서

- [docs/PROGRESS_REPORT.md](docs/PROGRESS_REPORT.md) — 전체 개발 진행 보고서, 튜닝 이력, 발산 분석
- [docs/ZENOH_UI_DEVELOPMENT_GUIDE.md](docs/ZENOH_UI_DEVELOPMENT_GUIDE.md) — 외부 UI 개발 가이드 (Zenoh 프로토콜)
- [docs/pipe_slam_research_summary.md](docs/pipe_slam_research_summary.md) — 파이프 환경 SLAM 연구 요약

## License

각 패키지 내 `package.xml` 참고.
