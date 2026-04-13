# 파이프 영상 평면화 알고리즘 개발 보고서

## 1. 프로젝트 개요

### 1.1 목적
모바일 로봇이 상수관로 내부에서 촬영한 ZED 카메라 영상을 2D 평면 이미지로 펼치는(unroll) 알고리즘 개발

### 1.2 데이터 소스
- **LiDAR**: Ouster LiDAR (`/ouster/points`) - 파이프 지오메트리 추정
- **카메라**: ZED X One Camera3 (`/zed_x_one/camera3/image_raw`) - 원본 영상
- **데이터 형식**: ROS2 bag 파일 재생 방식

### 1.3 출력
- `/pipe_vision/unrolled_image`: 표준 해상도 (400px 높이)
- `/pipe_vision/unrolled_image_raw`: 고해상도 (원본 이미지 높이 기준, 상하 반전)

---

## 2. 핵심 알고리즘

### 2.1 전체 파이프라인

```
LiDAR Point Cloud → 파이프 지오메트리 추정 → 좌표계 변환 → 이미지 평면화
```

### 2.2 단계별 상세 알고리즘

#### **Step 1: 파이프 지오메트리 추정 (RANSAC 기반 원 추정)**

**목적**: LiDAR 포인트 클라우드에서 파이프의 단면(원)을 추정하여 중심점과 반지름 계산

**알고리즘**:
1. **다중 슬라이스 샘플링**
   - x축 방향으로 13개 고정 위치에서 중심 추정
   - 가까운 영역 (0.5~2.0m): 5개 포인트
   - 먼 영역 (2.0~5.0m): 9개 포인트 (더 촘촘한 샘플링)
   - 비선형 분포로 멀리 있는 영역의 정확도 향상

2. **적응형 슬라이스 너비**
   - 가까운 거리: 0.4m 슬라이스
   - 먼 거리: 0.8m 슬라이스
   - 거리에 따라 더 많은 포인트로 안정성 확보

3. **RANSAC 원 추정**
   ```python
   def fit_circle_ransac(points_3d, x_min, x_max, x_center=None):
       # 거리 기반 적응형 파라미터
       if x_center > 2.5:  # 먼 거리
           residual_threshold = 0.18
           max_trials = 400
           min_inlier_ratio = 0.25
       else:  # 가까운 거리
           residual_threshold = 0.15
           max_trials = 300
           min_inlier_ratio = 0.3
       
       model, inliers = ransac(points_2d, CircleModel, ...)
       return (yc, zc, r)  # 중심점(y, z) 및 반지름
   ```

4. **반지름 필터링**
   - 합리적인 범위만 허용: 0.1m < r < 2.0m
   - 여러 슬라이스의 반지름 중앙값 사용

**출력**: 각 x 위치별 파이프 중심점 (yc, zc) 및 반지름 (r)

---

#### **Step 2: 좌표계 변환 및 중심점 평활화**

**목적**: LiDAR 좌표계에서 추정된 중심점을 base_link 좌표계로 변환하고 시간적 평활화 적용

**좌표계 변환**:
```
LiDAR Frame → base_link Frame
- 변환 행렬: R_lidar_bl, t_lidar_bl
- 하드코딩된 TF: t_bl_lidar = [0.0, 0.0, 0.32]
```

**시간적 평활화 (Exponential Moving Average)**:
- 각 x 위치별로 독립적으로 smoothing
- 기본 alpha = 0.01 (낮을수록 더 강한 smoothing)

**적응형 Smoothing**:
1. **거리 기반 smoothing**
   - 가까운 거리: alpha = 0.01
   - 먼 거리: alpha = 0.005 (2배 강한 smoothing)

2. **변화량 기반 적응형 smoothing**
   - 변화량 < 1.5cm: 기본 alpha 사용
   - 변화량 1.5~3cm: 선형 감소
   - 변화량 3~8cm: 지수 감소 (alpha = base × exp(-25 × (diff - 0.03)))
   - 변화량 > 8cm: 업데이트 스킵 (이전 값 유지)

**수식**:
```
smoothed_center[t] = alpha × current_center[t] + (1 - alpha) × smoothed_center[t-1]
```

**보정 오프셋**:
- 수동 보정: `calibration_offset = [0.0, 0.0, 0.25]` (z축 25cm)

---

#### **Step 3: 이미지 평면화 (Unrolling)**

**목적**: 원통형 파이프 표면을 2D 평면으로 매핑

**알고리즘**:

1. **평면화 이미지 그리드 생성**
   ```python
   # 표준 해상도
   unrolled_height = 400
   unrolled_width = int(2 * π * r_pipe * 400)
   
   # 고해상도 (raw)
   unrolled_height_raw = 원본_이미지_높이
   unrolled_width_raw = int(2 * π * r_pipe * (height / 4.5))
   ```

2. **3D 파이프 표면 점 생성**
   ```python
   theta = (u / width) * 2π  # 원주 방향 (0 ~ 2π)
   x_pipe = 0.5 + (v / height) * 4.5  # 길이 방향 (0.5m ~ 5.0m)
   
   # 중심점 보간 (Cubic Spline)
   yc = interp_y(x_pipe)  # y 중심 보간
   zc = interp_z(x_pipe)  # z 중심 보간
   
   # 파이프 표면 3D 점
   y_pipe = yc + r * cos(theta)
   z_pipe = zc + r * sin(theta)
   P_pipe = [x_pipe, y_pipe, z_pipe]
   ```

3. **좌표계 변환 체인**
   ```
   base_link → ROS Camera Frame → OpenCV Optical Frame
   ```
   - ROS 카메라 좌표계: X축 전방
   - OpenCV 광학 좌표계: Z축 전방
   - 변환 행렬: `R_cam_to_optical`

4. **카메라 투영**
   ```python
   image_points, _ = cv2.projectPoints(
       P_pipe_cam_cv, 
       rvec=np.zeros(3),  # 이미 카메라 좌표계에 있음
       tvec=np.zeros(3),
       K,  # 카메라 내부 파라미터
       D   # 왜곡 계수
   )
   ```

5. **이미지 리매핑**
   ```python
   unrolled_image = cv2.remap(
       original_image,
       map_x,  # x 좌표 맵
       map_y,  # y 좌표 맵
       cv2.INTER_LINEAR,
       borderMode=cv2.BORDER_CONSTANT
   )
   ```

**보간 방법**:
- 4개 이상 포인트: Cubic Spline 보간 (부드러운 곡선)
- 4개 미만: Linear 보간

---

## 3. 주요 개선 사항 및 해결 과정

### 3.1 초기 문제: S자 휘어짐

**문제**: 평면화된 이미지가 S자로 휘어짐

**원인 분석**:
- 단일 슬라이스(0.8~1.2m)만 사용하여 파이프 중심 추정
- 모든 x 위치에서 동일한 중심점 사용
- 파이프가 휘어져 있을 때 각 위치별 중심이 달라야 함

**해결 방법**:
1. 다중 슬라이스 중심 추정 (5개 → 9개 → 13개)
2. x 위치별 중심 보간 (Linear → Cubic Spline)
3. 적응형 smoothing 강화

### 3.2 중심 추정 불안정성

**문제**: 프레임마다 중심값이 크게 변동 (최대 10cm 이상)

**해결 방법**:
1. **적응형 Smoothing**
   - 변화량에 따라 alpha 자동 조정
   - 큰 변화(>8cm)는 스킵

2. **거리 기반 Smoothing**
   - 멀리 있을수록 더 강한 smoothing

3. **실패 처리**
   - 추정 실패 시 이전 프레임 값 유지
   - 전체 실패 시에도 이전 값 사용

### 3.3 멀리 있는 영역의 정확도 부족

**문제**: 카메라에서 멀어질수록 S자 휘어짐 증가

**해결 방법**:
1. **비선형 샘플링**
   - 멀리 있는 영역에 더 많은 샘플링 포인트

2. **적응형 슬라이스 너비**
   - 멀리 있을수록 더 넓은 슬라이스 (더 많은 포인트)

3. **거리 기반 RANSAC 파라미터**
   - 멀리 있을수록 더 관대한 파라미터

4. **Cubic Spline 보간**
   - 선형 보간 대신 더 부드러운 곡선

---

## 4. 기술적 세부사항

### 4.1 하드코딩된 TF 변환

```python
# base_link → Camera3
t_bl_cam = [0.15, 0.0, 0.12]  # meters
R_bl_cam = Identity (no rotation)

# base_link → LiDAR
t_bl_lidar = [0.0, 0.0, 0.32]  # meters
R_bl_lidar = Identity (no rotation)

# ROS Camera → OpenCV Optical
R_cam_to_optical = [[0, -1, 0],
                     [0, 0, -1],
                     [1, 0, 0]]
```

### 4.2 파라미터 설정

**RANSAC 파라미터**:
- `min_samples`: 5
- `residual_threshold`: 0.15 (near) / 0.18 (far)
- `max_trials`: 300 (near) / 400 (far)
- `min_inlier_ratio`: 0.3 (near) / 0.25 (far)

**Smoothing 파라미터**:
- 기본 alpha: 0.01
- 거리 기반 감소: 멀리 있을수록 0.5배
- 변화량 기반 감소: 지수 감쇠 (k=25.0)

**평면화 파라미터**:
- x축 범위: 0.5m ~ 5.0m
- 표준 해상도: 400px 높이
- 고해상도: 원본 이미지 높이

### 4.3 메시지 동기화

```python
# ApproximateTimeSynchronizer 사용
synchronizer = ApproximateTimeSynchronizer(
    [pc_sub, image_sub],
    queue_size=10,
    slop=0.5  # 0.5초 허용 오차
)
```

### 4.4 에러 처리

1. **RANSAC 실패**: 이전 프레임 값 사용
2. **전체 실패**: 모든 슬라이스 실패 시 이전 smoothed 값 사용
3. **부족한 포인트**: 최소 3개 이상 필요 (보간을 위해)
4. **비정상 반지름**: 0.1m ~ 2.0m 범위만 허용

---

## 5. 성능 및 결과

### 5.1 안정성 개선

- **초기**: 중심값 변화 최대 10cm 이상
- **개선 후**: 변화량 3cm 이상 시 적응형 smoothing, 8cm 이상 시 스킵
- **결과**: S자 휘어짐 거의 제거

### 5.2 정확도 개선

- **초기**: 5개 샘플링 포인트, 선형 보간
- **개선 후**: 13개 샘플링 포인트 (멀리 있는 영역 집중), Cubic Spline 보간
- **결과**: 멀리 있는 영역의 정확도 향상

### 5.3 해상도

- **표준**: 400px 높이 (빠른 처리)
- **고해상도**: 원본 이미지 높이 (1200px) - 상세 분석용

---

## 6. 알고리즘 특징

### 6.1 장점

1. **실시간 처리**: ROS2 bag 재생 시 실시간 평면화
2. **안정성**: 적응형 smoothing으로 노이즈 제거
3. **정확도**: 다중 샘플링 및 Cubic Spline 보간
4. **견고성**: 실패 시 이전 값 유지로 연속성 보장

### 6.2 제한사항

1. **하드코딩된 TF**: 실제 TF 트리를 사용하지 않음
2. **고정된 x 범위**: 0.5m ~ 5.0m 범위만 처리
3. **원형 파이프 가정**: 원형 단면만 지원
4. **수동 보정**: calibration_offset 수동 조정 필요

---

## 7. 향후 개선 방향

1. **TF 트리 활용**: 하드코딩 대신 실제 TF 트리 사용
2. **동적 x 범위**: 파이프 길이에 따라 자동 조정
3. **파라미터화**: ROS2 파라미터로 모든 하드코딩 값 변경
4. **연속 파노라마**: 오도메트리 기반 이미지 스티칭
5. **비원형 파이프**: 타원형 등 다른 단면 형태 지원

---

## 8. 참고 자료

### 8.1 주요 라이브러리
- `scikit-image`: RANSAC 원 추정
- `scipy`: 보간 (interp1d)
- `OpenCV`: 이미지 처리 및 투영
- `ROS2`: 메시지 처리 및 동기화

### 8.2 관련 파일
- `pipe_vision/pipe_unroller_node.py`: 메인 알고리즘 구현
- `pipe_vision/inspect_tf.py`: TF 프레임 확인 유틸리티
- `log/pipe_vision/`: 디버깅 로그 파일

---

**작성일**: 2025년 11월 19일  
**버전**: 최종 개선 버전











