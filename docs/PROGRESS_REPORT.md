# VILL-SLAM 프로젝트 진행 상황 보고서
**최종 업데이트: 2026-04-10**

---

## 1. 하드웨어 센서 구성

### 현재 연결 상태
| 센서 | 모델 | 연결 상태 | 비고 |
|------|------|----------|------|
| LiDAR | Ouster OS1-128 | ✅ 정상 | IP: 169.254.241.98, eno1에 169.254.241.1/24 추가 필요 |
| 전방 카메라 | ZED X (S/N 40782131) | ✅ 정상 | GMSL Port 0, i2c-9 |
| 좌측 카메라 | ZED X One 4K (S/N 319430526) | ✅ 정상 | GMSL Port 2, i2c-10 |
| IMU (내장) | Ouster IMU | ✅ 사용중 | BEST_EFFORT QoS, 100Hz |
| IMU (외장) | MicroStrain GV7-AHRS | ✅ 설치됨 | /dev/microstrain_main, 미사용 |
| GNSS | u-blox | ✅ 연결 | /dev/ublox_gps, 미사용 |

### ZED Link Duo 캡처카드 제한
- **ZED X One 4K(UHD)는 최대 2대**까지 동시 스트리밍 (대역폭 제한)
- 드라이버: stereolabs-zedlink-duo v1.4.1 (LI-MAX96712-L4T36.4.0)
- SDK: ZED SDK 5.2.3
- 우측 카메라 추가 시 **ZED Link Quad 보드로 교체 필요**
- ZED X One 4K 포트 규칙: Port #0, #2만 허용 (v1.4.1)

### 시스템 사양
- **플랫폼**: NVIDIA Jetson AGX Orin Developer Kit
- **OS**: Ubuntu 22.04, L4T 36.4.4 (JetPack 6.2.1)
- **ROS**: ROS2 Humble
- **GPU**: Orin (30.7GB, CUDA 12.6)

---

## 2. 소프트웨어 구성

### 빌드 완료 패키지
| 패키지 | 상태 | 용도 |
|--------|------|------|
| `coin_lio` | ✅ 빌드 완료 | COIN-LIO ROS2 (메인 SLAM 오도메트리) |
| `fast_lio` | ✅ 빌드 완료 | Fast-LIO2 ROS2 (백업) |
| `fast_livo` | ✅ 빌드 완료 | FAST-LIVO2 (nan 초기화 문제 미해결) |
| `vill_slam` | ✅ 빌드 완료 | VILL-SLAM (루프 클로저 + Dense RGB-D 매핑) |
| `ouster_ros` | ✅ 빌드 완료 | Ouster LiDAR 드라이버 |
| `zed_wrapper` | ✅ 빌드 완료 | ZED 카메라 드라이버 (SDK 5.2.3 호환) |
| `zed_description` | ✅ 빌드 완료 | ZED URDF 모델 |

### COIN-LIO ROS2 포팅 내역
- **소스**: `/home/test/ros2_ws/src/coin_lio/`
- **기반**: Fast-LIO2 ROS2 + COIN-LIO (ETH Zurich, ICRA 2024) photometric 모듈
- **포팅 완료 파일**:
  - `laserMapping.cpp` - Fast-LIO2 ROS2 기반 + photometric 함수 3개 병합
  - `feature_manager.h/cpp` - 반사강도 특징점 추적/감지 (ROS2)
  - `image_processing.h/cpp` - 반사강도 이미지 처리 파이프라인 (ROS2)
  - `projector.h/cpp` - Ouster LiDAR 투영 모델 (ROS2)
  - `timing.h/cpp` - 성능 프로파일링
  - `common_lib.h` - LidarFrame, V2D, V4D, M4D, Pose6D, getSubPixelValue 추가
  - `CMakeLists.txt`, `package.xml` - OpenCV 의존성 포함

---

## 3. SLAM 테스트 결과

### 3.1 Fast-LIO2 (LiDAR + IMU only)
| 항목 | 결과 |
|------|------|
| 정지 상태 | ✅ 안정 (x=-0.14, y=0.01, z=0.006) |
| 짧은 직진+후진 | ✅ 안정 (z ±0.3m) |
| 직진+선회+후진 | ⚠️ acc_cov=10 설정 시 성공, 기본값 시 발산 |
| 장거리 복도 주행 | ❌ z축 드리프트 → 발산 |
| **결론** | Ouster 내장 IMU 품질 한계로 장거리 불안정 |

### 3.2 FAST-LIVO2 (LiDAR + Camera + IMU)
| 항목 | 결과 |
|------|------|
| 카메라 파라미터 로딩 | ✅ 해결 (declare_parameter 추가) |
| 이미지 QoS | ✅ 해결 (RELIABLE로 변경) |
| LIO effective features | ❌ 항상 0 (nan 초기화 실패) |
| **결론** | 초기 포즈 추정 실패, 근본 원인 미해결 |

### 3.3 COIN-LIO (LiDAR + Intensity + IMU) ⭐ 현재 사용
| 항목 | 결과 |
|------|------|
| 기본 동작 (photo_scale=0) | ✅ 정상 |
| 정지 상태 | ✅ 안정 |
| 복도 한바퀴 주행 | ✅ **발산 없이 완주!** |
| z축 안정성 | ✅ ±0.3m 이내 |
| 루프 클로저 | ❌ 미지원 (드리프트 있음) |
| Ouster 메타데이터 로딩 | ✅ 구현 완료 (2026-04-10) |
| photometric 모드 설정 | ✅ photo_scale=0.00095, FIR 필터 통합 완료 |
| photometric 모드 실주행 | ⏳ 테스트 필요 |
| **결론** | 가장 안정적. photometric 모드 준비 완료, 실주행 검증 대기 |

---

## 4. 해결된 주요 이슈

### 4.1 카메라 관련
- [x] ZED X One 4K 교체 후 GMSL 포트 충돌 → 포트 배치 해결
- [x] ZED Link Duo i2c bus 중복 → 물리 포트 재배치
- [x] ZED X One 4K 2대 제한 확인 (Duo 보드 하드웨어 한계)
- [x] zed-ros2-wrapper SDK 5.2.3 호환 업데이트
- [x] zed_description 패키지 추가
- [x] sensors.launch.py 시리얼번호/모델 매칭 수정

### 4.2 LiDAR 관련
- [x] Ouster 네트워크 IP 설정 (169.254.241.1/24)
- [x] Ouster lidar_mode 1024x10 ↔ 512x10 전환
- [x] ring 필드 타입 불일치 (uint8 vs uint16) - 원본 유지가 안정적

### 4.3 IMU 관련
- [x] QoS 불일치: Ouster(BEST_EFFORT) vs ZED(RELIABLE) → SensorDataQoS 사용
- [x] MicroStrain GV7 드라이버 설치 + udev 규칙 고정
- [x] GV7 PPS/declination 설정 오류 → 비활성화

### 4.4 SLAM 관련
- [x] Fast-LIO2 IMU covariance 튜닝 (acc_cov=10, gyr_cov=10)
- [x] FAST-LIVO2 camera params 로딩 (declare_parameter)
- [x] FAST-LIVO2 image QoS RELIABLE 변경
- [x] COIN-LIO ROS2 포팅 완료
- [x] Feature 이름 충돌 해결 (Feature → PhotoFeature)
- [x] common_lib.h multiple definition → inline 추가
- [x] COLCON_IGNORE로 ROS1 패키지 충돌 방지

### 4.5 VILL-SLAM 루프 클로저 + Dense RGB-D (2026-04-10)
- [x] `vill_slam_node.cpp` 루프 클로저 구현 (GTSAM iSAM2 + ICP)
- [x] 키프레임 관리 (거리/회전 임계값 기반 자동 선택)
- [x] 포즈 그래프에 Cylinder/Plane Factor 통합
- [x] `dense_mapper_node.cpp` RGB 색상 투영 구현
- [x] CameraInfo 자동 로딩 + LiDAR→카메라 외부 파라미터
- [x] `vill_slam_params.yaml`에 루프 클로저/Dense 매핑 파라미터 추가
- [x] 통합 빌드 성공 확인

### 4.6 Recorder 발산 분석 인프라 (2026-04-10)
- [x] `multi_format_recorder.py` 확장: SLAM 성능 모니터링 데이터 추가 기록
- [x] **3종 CSV 동시 기록**:
  - `robot_data_<ts>.csv` — 기존 휠/SLAM/환경 (10Hz)
  - `slam_metrics_<ts>.csv` — 발산 분석용 40+ 컬럼 (10Hz)
  - `loop_closures_<ts>.csv` — 루프 클로저 이벤트 로그
- [x] **40+ metrics 기록**:
  - Raw vs Corrected 포즈 비교 (COIN-LIO `/Odometry` 직접 구독)
  - 프레임 간 점프 (`raw_step`, `corr_step`, `raw_jump`)
  - Pose covariance (`pose_cov_xyz`, `pose_cov_rot`)
  - IMU 가속/각속도 (`imu_acc_*`, `imu_gyro_*`, `imu_acc_norm`)
  - LiDAR 포인트 수 (`lidar_point_count`)
  - 토픽 주파수 (`lidar_hz`, `imu_hz`, `cam_front_hz`, `odom_hz`)
  - 시스템 리소스 (`cpu_pct`, `mem_pct` via psutil)
  - VILL-SLAM 통계 (keyframes, constraints, loop_closure_count, drift)
- [x] **루프 클로저 이벤트 자동 감지** + 콘솔 출력 (`[LOOP] #N @ pos correction=Xm`)
- [x] `vill_slam_node.cpp` `publishStatus()` 확장: loop_closure_count, num_constraints, estimated_drift 채움

### 4.6 COIN-LIO Photometric 모드 준비 (2026-04-10)
- [x] Ouster 메타데이터 로딩 구현 (`projector.cpp` → ROS2 파라미터로 로딩)
  - `beam_altitude_angles` (128개 빔 고도각) - `ouster128.yaml`에 설정
  - `pixel_shift_by_row` (128개 destagger 오프셋) - `ouster128.yaml`에 설정
  - `lidar_origin_to_beam_origin_mm: 15.806` - 빔 오프셋
- [x] `line_removal.yaml` FIR 필터 계수 통합 (별도 YAML → launch에서 로딩)
  - highpass 33탭 + lowpass 33탭 FIR 커널
- [x] `photo_scale: 0.00095` 설정 (photometric 모드 활성화)

---

## 5. 앞으로 해야 할 일

### 5.1 단기 (COIN-LIO photometric 활성화)
- [x] **Ouster 메타데이터 로딩 구현** (2026-04-10 완료)
  - `beam_altitude_angles` (128개 빔 각도) → `ouster128.yaml`
  - `pixel_shift_by_row` (128개 픽셀 오프셋) → `ouster128.yaml`
  - ROS2 파라미터로 로딩 (`projector.cpp:loadParameters()`)
- [x] `line_removal.yaml` FIR 필터 계수 통합 (별도 YAML, launch에서 자동 로딩)
- [x] `photo_scale=0.00095` 설정 완료 (photometric 모드 활성화)
- [ ] **photometric 모드 실주행 테스트** (복도 직선 구간 드리프트 개선 확인)

### 5.2 중기 (VILL-SLAM 연동)
- [x] COIN-LIO `/Odometry` 출력 → VILL-SLAM 입력 연결 (토픽 매칭 확인 완료)
- [x] 통합 launch 파일 업데이트 (`vill_slam_integrated.launch.py`)
  - 센서 드라이버 → COIN-LIO (3초 딜레이) → VILL-SLAM (5초 딜레이) 순차 실행
  - `launch_sensors:=false` + `use_sim_time:=true`로 rosbag 재생 지원
  - `coin_lio_config` 인자로 COIN-LIO 설정 파일 선택 가능
- [x] **루프 클로저 구현** (2026-04-10)
  - GTSAM iSAM2 기반 증분 포즈 그래프 최적화
  - 키프레임 관리: 거리(0.5m)/회전(0.3rad) 기반 자동 선택
  - ICP 스캔 매칭으로 루프 후보 검증 (fitness < 0.3)
  - Huber robust noise model로 false positive 방지
  - Cylinder/Plane Factor 포즈 그래프 통합
  - `/vill_slam/path` 토픽으로 최적화된 경로 발행
- [x] **Dense RGB-D 매핑 구현** (2026-04-10)
  - ZED X 카메라 RGB 이미지로 LiDAR 포인트 색상화
  - CameraInfo에서 intrinsic 자동 로딩 (fx, fy, cx, cy)
  - LiDAR→카메라 외부 파라미터(T_cam_lidar) 설정 가능
  - 투영 실패 시 높이 기반 색상 fallback
  - PCD 파일 저장 서비스 (`/vill_slam/save_dense_map`)
- [x] **VILL-SLAM 복도 모드 1차 주행 테스트 완료** (2026-04-10)
- [x] **루프 클로저 동작 확인** — 5.5분 주행 중 35회 발생, 평균 보정 0.15m
- [ ] Dense RGB-D 색상 정확도 확인 (카메라-LiDAR 캘리브레이션 필요)

### 5.2.1 1차 주행 테스트 결과 분석 (2026-04-10)

**테스트 조건**: 복도 한 바퀴 주행, 5.5분 (331초), AUTO 모드

#### ✅ 정상 동작 확인
| 항목 | 결과 |
|------|------|
| 영상 녹화 | ✅ front/left mp4 정상 (973/758 frames) |
| CSV 데이터 | ✅ robot_data + slam_metrics + loop_closures 3종 정상 |
| Zenoh UI 통신 | ✅ pose/path/dense_map 송신 |
| 루프 클로저 | ✅ 35회 발생 (보정량 평균 0.15m, 정상 범위) |
| GTSAM 포즈 그래프 | ✅ keyframe 98개, factor graph 정상 작동 |
| Recorder 서비스 | ✅ UI Zenoh START/STOP_RECORD 정상 |

#### ❌ 발산 발생 (t=325.5s 이후)

**발산 시퀀스 (분석 데이터: `slam_metrics_20260410_143223.csv`)**:
```
t=  0~313s : 정상 주행 (raw_step ~0.01-0.07m)
t=313~325s : z축 서서히 부유 (z: 0.20→0.78m, 5초간 +0.58m) ← IMU drift 누적
t=325.5s   : raw_step 갑자기 0.40→0.51→0.65m 점프 (ICP 빗나감)
t=327.2s   : 마지막 루프 클로저 #35 — 보정량 3.717m (정상 30배)
t=327~331s : raw odometry 도미노 발산 (z축 -9m 점프)
```

**근본 원인 (확정)**: **LiDAR 토픽 처리 병목**

| 토픽 | 정상 | 측정값 | 손실율 |
|------|------|--------|--------|
| LiDAR (`/ouster/points`) | 10 Hz | **평균 6.1 Hz** (min 0.8) | 40% |
| Odometry (`/Odometry`) | 10 Hz | 7.3 Hz | 27% (LiDAR 부족 결과) |
| IMU (`/ouster/imu`) | 100 Hz | 99.9 Hz | 0% (정상) |
| Camera (`/zed_front/.../image`) | 15 Hz | 9.8 Hz | 35% |

LiDAR 입력 부족 → ESIKF가 IMU 적분에 과의존 → z축 bias drift 누적 → ICP 빗나감 → 발산.

**의심 원인**: LiDAR 토픽을 4개 노드가 동시 구독
1. `coinlio_mapping` (메인 SLAM)
2. `vill_slam_node` (키프레임용 voxel filter)
3. `dense_mapper_node` (매 프레임 카메라 투영 + 점 단위 색상화)
4. `multi_format_recorder` (Hz 카운터 — 가벼움)

DDS BEST_EFFORT QoS + 큐 오버플로 + dense_mapper의 무거운 픽셀 투영 → 메시지 drop.

CPU도 가끔 105%(1코어 초과) 찍음.

### 5.2.2 발산 해결 계획 (진행 중)

#### 1순위 — LiDAR 처리 throttle ✅ 완료 (2026-04-10)
- [x] **`dense_mapper_node`**: `cloudCallback`에 throttle 추가 (`process_rate_hz: 1.5`)
- [x] **`vill_slam_node`**: `lidarCallback`에 throttle 추가 (`lidar_process_rate_hz: 1.0`)
- [x] 빌드 성공 + 재주행 검증
- [x] **결과**: lidar_hz 6.1 → 7.6 Hz 개선, 발산 빈도 감소 (run #1: 5분 거의 안정)
  - 단, 부하가 낮은 시간에는 발산 거의 없음 (run #1: 발산 거의 없음, 91 loops/106 keyframes)
  - 부하 높을 때 여전히 발산 (run #2: 174 loops/139 keyframes, 빠른 발산)
  - **핵심 원인 재진단**: 진짜 문제는 LiDAR Hz가 아니라 **루프 클로저 false positive 폭주**

#### 1.5순위 — 루프 클로저 검증 강화 ✅ 완료 (2026-04-10)

**진단**: 두 번째 run에서 174 loops / 139 keyframes (125%) 발견. 매 키프레임마다 루프 클로저 발생 = false positive 폭주. corr_step이 매 0.3초마다 0.3-1.3m로 점프하면서 잘못된 보정으로 발산.

**적용 변경**:

| 파라미터 | 이전 | 변경 | 효과 |
|---------|------|------|------|
| `icp_fitness_threshold` | 0.3 | **0.1** | 훨씬 엄격한 ICP 매칭만 인정 |
| `loop_search_radius` | 5.0m | **3.0m** | 너무 먼 후보 제외 |
| `loop_min_keyframe_interval` | 30 | **50** | 시간적으로 더 멀어야 루프로 인정 |
| `loop_cooldown_keyframes` | (없음) | **5** | 신규: 최근 루프 후 N개 키프레임 동안 다음 루프 금지 |

- [x] yaml 파라미터 4개 변경 (`vill_slam_params.yaml`)
- [x] `vill_slam_node.cpp`에 cooldown 로직 추가
  - `loop_cooldown_active_`, `last_loop_kf_id_` 멤버 변수
  - `detectLoopClosure()`에서 조기 return 체크
  - 루프 발생 시 cooldown 활성화
  - reset 서비스에서도 cooldown 상태 초기화
- [x] 빌드 성공
- [ ] **재주행 검증** (다음 단계): 루프 클로저 비율(loops/keyframes)이 5-30%로 떨어지는지 확인

#### 보조 발견: IMU 진동 비정상 (실제 환경 확인됨)
- 두 번째 run에서 `imu_acc_norm` 12, 14, 17 등 비정상 값 (정상 9.8±1)
- **사용자 확인**: 실제로 로봇 진동이 심함 (환경 문제)
- 향후 2순위 작업: COIN-LIO `acc_cov` 튜닝 (10 → 50)으로 IMU 신뢰도 낮추기

#### 2순위 — z축 IMU drift 방지
- [ ] COIN-LIO `acc_cov` 튜닝 (10 → 50?) IMU 신뢰도 낮춤
- [ ] 또는 z축 prior factor 추가 (수평 환경 가정)
- [ ] photometric 모드(`photo_scale`)가 부하의 일부일 가능성 → 0으로 일시 비활성화 후 비교 테스트

#### 3순위 — 시스템 부하 분산
- [ ] CPU 사용량 노드별 분석 (`top -H -p $(pgrep -d, coinlio_mapping)`)
- [ ] dense_mapper 외부 분리 가능성 (별도 launch에서 띄우기)

#### 검증 방법
- 1순위 적용 후 동일한 복도 한 바퀴 주행
- `slam_metrics_*.csv`에서 `lidar_hz`가 9-10 Hz 유지되는지 확인
- 5분 이상 주행 시 발산 없는지 확인 (특히 z축)

### 5.3 장기 (시스템 완성)
- [ ] ZED Link Quad 보드 교체 → 우측 카메라 추가
- [ ] 파이프 환경 테스트 (`environment:=pipe`)
- [ ] 레이저 프로파일러 연동 (파이프 단면 측정)
- [ ] Bunker Mini CAN 연결 → 휠 오도메트리 융합
- [ ] 전체 시스템 통합 테스트 (센서 + SLAM + 매핑 + 레코딩)

---

## 6. 실행 명령어 요약

### 센서 드라이버
```bash
# Ouster IP 설정 (재부팅 후 필요)
sudo ip addr add 169.254.241.1/24 dev eno1

# 센서 실행
source install/setup.bash
ros2 launch vill_slam sensors.launch.py
```

### COIN-LIO SLAM
```bash
source install/setup.bash
ros2 launch coin_lio mapping.launch.py
# RViz: Fixed Frame = camera_init, Add PointCloud2 /cloud_registered, Path /path
```

### 실시간 모니터링
```bash
bash /home/test/ros2_ws/log_slam_status.sh
# 1분간 2초 간격으로 위치 기록
```

### 전체 시스템 통합 실행 (센서 + COIN-LIO + VILL-SLAM)
```bash
source install/setup.bash

# 복도 모드 (기본)
ros2 launch vill_slam vill_slam_integrated.launch.py

# 파이프 모드
ros2 launch vill_slam vill_slam_integrated.launch.py environment:=pipe

# rosbag 재생 시 (센서 없이)
ros2 launch vill_slam vill_slam_integrated.launch.py launch_sensors:=false use_sim_time:=true

# 최소 구성 (UI/레코더 없이)
ros2 launch vill_slam vill_slam_integrated.launch.py use_ui:=false use_recorder:=false
```

### MicroStrain GV7 (필요 시)
```bash
ros2 launch microstrain_inertial_driver microstrain_launch.py \
  params_file:=/home/test/ros2_ws/src/vill_slam/config/microstrain_gv7.yaml
```

---

## 7. 주요 파일 경로

| 항목 | 경로 |
|------|------|
| COIN-LIO 소스 | `/home/test/ros2_ws/src/coin_lio/` |
| COIN-LIO config | `/home/test/ros2_ws/src/coin_lio/config/ouster128.yaml` |
| Fast-LIO2 소스 | `/home/test/ros2_ws/src/fast_lio/` |
| FAST-LIVO2 소스 | `/home/test/ros2_ws/src/FAST-LIVO2/` |
| VILL-SLAM 소스 | `/home/test/ros2_ws/src/vill_slam/` |
| 통합 launch | `/home/test/ros2_ws/src/vill_slam/launch/vill_slam_integrated.launch.py` |
| 센서 launch | `/home/test/ros2_ws/src/vill_slam/launch/sensors.launch.py` |
| ZED front config | `/home/test/ros2_ws/src/vill_slam/config/zed_front_params.yaml` |
| Ouster config | `/home/test/ros2_ws/src/vill_slam/config/ouster_params.yaml` |
| GV7 config | `/home/test/ros2_ws/src/vill_slam/config/microstrain_gv7.yaml` |
| udev rules | `/etc/udev/rules.d/99-robot-sensors.rules` |
| SLAM 로그 | `/home/test/ros2_ws/slam_logs/` |
| 로그 스크립트 | `/home/test/ros2_ws/log_slam_status.sh` |
| COIN-LIO ROS1 참조 | `/home/test/ros2_ws/scout_mini/code/catkin_ws/src/COIN-LIO/` |
