# 상수관로 탐사용 자율주행 이동체의 SLAM/위치추정 관련 논문 조사 보고서

> 조사 일자: 2026-04-02 | 대상 기간: 2022~2025 | 키워드: In-Pipe SLAM, Featureless Environment, Localization, Multi-Sensor Fusion

---

## 1. 연구 배경 및 핵심 과제

상수관로 내부는 SLAM 기술 적용에 있어 다음과 같은 **극한 환경(Extreme Environment)** 특성을 가진다:

- **특징점 부족(Feature-sparse)**: 관로 내벽은 균일한 텍스처로 이루어져 있어 시각적 특징점(corner, edge, texture)이 극히 제한적
- **자기 유사성(Self-similarity)**: 직선 구간이 반복되어, 다른 위치임에도 센서 데이터가 거의 동일하게 관측됨
- **GPS 단절**: 지하 매설 환경으로 위성 신호 수신 불가
- **제한된 공간**: 소구경 관로(75~300mm)에서는 센서 배치 및 로봇 크기에 물리적 제약
- **조명 부족**: 인공 조명에 전적으로 의존하며, 반사와 그림자가 비전 시스템에 간섭

이 같은 환경은 상수관로뿐 아니라 **터널, 갱도, 하수관로, 해저 파이프라인** 등에서도 공통적으로 나타나며, 기존의 범용 SLAM 알고리즘(ORB-SLAM2, VINS-Mono, LOAM 등)이 정상적으로 동작하지 않는 주된 원인이 된다.

---

## 2. 핵심 논문 분석

### 2.1 VILL-SLAM: 관로 전용 다중센서 융합 SLAM

| 항목 | 내용 |
|------|------|
| **논문명** | Visual-Inertial-Laser-Lidar (VILL) SLAM: Real-Time Dense RGB-D Mapping for Pipe Environments |
| **저자** | Tina Tian, Lu Wang, Xin Yan, Fuheng Ruan, G. J. Aadityaa, Howie Choset, Lu Li |
| **소속** | Carnegie Mellon University (CMU), Biorobotics Lab |
| **발표** | IROS 2023 (IEEE/RSJ Int. Conf. Intelligent Robots and Systems) + 2025 석사학위논문으로 확장 |
| **DOI** | 10.1109/IROS55552.2023.10341761 |

#### 핵심 아이디어

VILL-SLAM은 **관로 환경에 특화된 compact 센서 패키지**를 구성하고, 이를 tightly-coupled sliding-window 기반 SLAM 파이프라인에 통합한 시스템이다.

**센서 구성 (4종 융합)**:
- **V (Visual)**: 어안 렌즈 장착 RGB 카메라 — 넓은 시야각 확보
- **I (Inertial)**: MEMS 기반 6축 IMU — 관성 항법
- **L (Laser)**: 원뿔형 거울을 이용한 링 형태 레이저 프로파일러 — 관로 내벽 단면 형상 추출
- **L (LiDAR)**: Realsense L515 — 3D 점군 데이터

**알고리즘 핵심**:
1. **Alternating-shutter 기법**: 레이저 링과 일반 카메라 프레임을 교대 촬영하여 간섭 없이 두 종류 데이터를 동시 획득
2. **레이저 삼각측량(Laser Triangulation)**: 레이저 링으로부터 관로 내벽까지의 정밀 거리 측정
3. **관로 원통 기하학적 제약(Cylinder Factor)**: 관로가 원통형이라는 사전 지식을 factor graph에 제약 조건으로 추가하여 장거리 drift 억제
4. **Dense RGB-D 출력**: 최종 결과물은 관로 내부의 컬러 3D 점군 맵

**실험 결과**: 12인치 직경 천연가스 관로에서 50m 구간 테스트 시, **기존 SOTA 알고리즘(VINS-Mono, LOAM, FAST-LIO2, ORB-SLAM2, SSL-SLAM2, RGBDTAM)은 모두 합리적인 결과를 생성하지 못했으나**, VILL-SLAM만이 정밀한 localization과 고품질 3D 맵핑에 성공.

#### 프로젝트 적용 시사점

- 상수관로 역시 원통형 구조이므로 **cylinder factor 기반 drift 보정 전략**을 직접 참고 가능
- 레이저 프로파일러가 시각 특징점을 대체하는 핵심 역할을 수행 — **특징점 부족 환경에 대한 실질적 해법**
- 센서 패키지가 소형화되어 있어 소구경 관로 적용 가능성 높음

---

### 2.2 Robust-LIWO: 터널 환경 특화 LiDAR-IMU-Wheel 융합 SLAM

| 항목 | 내용 |
|------|------|
| **논문명** | Efficient and accurate 3D reconstruction in a featureless tunnel environment: a robust LiDAR-Inertial SLAM tightly coupled with wheel odometry |
| **학술지** | Journal of Civil Structural Health Monitoring (Springer, 2025) |
| **DOI** | 10.1007/s13349-025-01005-w |

#### 핵심 아이디어

터널 환경의 3D 복원을 위해 **LiDAR + IMU + Wheel Odometry**를 ESKF(Error State Kalman Filter)로 tightly-coupled 융합한 시스템.

**주요 기여**:
1. **Degeneracy Detection**: 점군 제약 관계를 분석하여 LiDAR 관측이 퇴화(degenerate)되는 상황을 실시간으로 감지
2. **Dual-LiDAR Fusion**: 두 개의 LiDAR를 결합하여 단일 센서의 한계를 보완하고 고품질 3D 재구성 달성
3. **Wheel Odometry 통합**: GPS 단절 환경에서 wheel encoder가 중력 방향 이동 및 직선 구간 drift 보정에 기여

#### 프로젝트 적용 시사점

- 상수관로 탐사 로봇에 **wheel encoder가 이미 장착되어 있다면**, 이를 SLAM에 직접 융합하는 전략이 매우 효과적
- **Degeneracy detection**은 관로의 긴 직선 구간에서 LiDAR 포인트가 퇴화되는 현상에 대비할 수 있는 핵심 모듈

---

### 2.3 Modified FAST-LIO2: 특징점 없는 터널에서의 휠 인코더 융합 SLAM

| 항목 | 내용 |
|------|------|
| **논문명** | LiDAR SLAM with a Wheel Encoder in a Featureless Tunnel Environment |
| **저자** | I. Filip, J. Pyo, M. Lee, H. Joe |
| **학술지** | Electronics, 2023, 12(4), 1002 (MDPI, Open Access) |
| **선행 연구** | Lidar SLAM Comparison in a Featureless Tunnel Environment (ICCAS 2022) |

#### 핵심 아이디어

**FAST-LIO2를 베이스라인으로 선택한 이유**: 특징점 기반 알고리즘(LOAM, LeGO-LOAM 등)은 edge feature가 거의 없는 터널 환경에서 성능이 크게 저하되는 반면, FAST-LIO2는 raw point를 직접 사용하는 **direct 방식**이라 특징점 부족 환경에서도 상대적으로 강건함.

**개선 방법**:
- FAST-LIO2의 LiDAR odometry 출력과 wheel encoder 데이터를 **EKF(Extended Kalman Filter)**로 융합
- Prediction 단계: wheel encoder + IMU 데이터 사용
- Correction 단계: FAST-LIO2 LiDAR 상태 추정값으로 보정
- EKF는 LiDAR odometry 추정 이후, 맵핑 프로세스 이전에 삽입

**실험 결과**: 수평 및 경사면 터널 환경 모두에서 기존 FAST-LIO2 대비 매핑 정확도 및 localization 성능이 유의미하게 향상됨.

**선행 비교 연구(ICCAS 2022)의 주요 결론**:
- 7종 LiDAR SLAM 비교 결과, 터널 환경에서 **LIO-SAM과 FAST-LIO2가 가장 우수**
- 인공 랜드마크(edge feature) 추가 시 feature 기반 SLAM 성능이 크게 향상되나, direct 방식은 영향이 상대적으로 작음

#### 프로젝트 적용 시사점

- **FAST-LIO2 + wheel encoder**는 비교적 간단한 구현으로 상수관로 같은 featureless 환경에서 실질적인 성능 향상 가능
- 인공 랜드마크 전략도 고려 가능 — 관로 내부에 특정 간격으로 식별 가능한 마커를 설치하여 loop closure 지원

---

### 2.4 CompSLAM: 지하 환경 자율 로봇을 위한 계층적 다중모달 SLAM

| 항목 | 내용 |
|------|------|
| **논문명** | CompSLAM: Complementary Hierarchical Multi-Modal Localization and Mapping for Robot Autonomy in Underground Environments |
| **관련** | DARPA Subterranean Challenge (SubT) 참가팀 연구 |
| **발표** | 2024~2025 |

#### 핵심 아이디어

DARPA SubT Challenge에서 검증된 시스템으로, 터널·동굴·도시 지하 구조물 등 **다양한 지하 환경**에서의 robust SLAM을 목표로 한다.

**핵심 구조: 계층적 보완 시스템(Complementary Hierarchy)**:
1. **1차 Odometry**: Visual-Tactile-Inertial Odometry (VTIO) — 카메라 + IMU 기반
2. **2차 보완**: LiDAR 기반 점군 정합 — 시각 특징이 부족할 때 LiDAR가 보완
3. **3차 Fallback**: 다리 odometry(ANYmal 사지보행 로봇의 경우) 또는 wheel encoder — 모든 센서가 퇴화될 때 최후 수단
4. **계층적 정제**: 각 odometry 소스의 추정값이 다음 모듈의 초기값으로 전달되어 점진적으로 정밀도 향상

**Self-similar 터널 대응**: 반복 구조의 긴 터널에서 visual odometry와 LiDAR 모두 유사한 관측을 반환하여 loop closure false positive가 발생하는 문제를 계층적 검증으로 억제.

#### 프로젝트 적용 시사점

- **센서 퇴화(degradation)에 대한 계층적 대응 전략**은 상수관로 로봇에도 필수적
- 단일 센서 의존 시스템은 관로 환경에서 반드시 실패하므로, **다중 센서 간 우선순위 기반 전환(fallback)** 구조 설계 필요

---

### 2.5 딥러닝 기반 Loop Closure Detection for 고유사도 환경

| 항목 | 내용 |
|------|------|
| **논문명** | Accurate localization of indoor high similarity scenes using visual SLAM combined with loop closure detection algorithm |
| **학술지** | PLoS ONE, 2024, 19(12):e0312358 |
| **소속** | Changchun University of Science and Technology |

#### 핵심 아이디어

**시각적으로 유사한 실내 환경**(긴 복도, 반복 구조 홀)에서의 visual SLAM 정확도를 딥러닝 기반 loop closure detection으로 개선.

**주요 기여**:
1. **딥러닝 기반 특징 추출**: 기존 handcrafted feature(ORB, SIFT) 대신 CNN 기반의 의미론적(semantic) 특징 추출
2. **오매칭 감소**: 시각적으로 유사한 장면에서 발생하는 false positive loop closure를 딥러닝 모델의 복합 의미 분석으로 감소
3. **하이브리드 매칭**: 개선된 feature 추출과 기하학적 검증을 결합

**실험 데이터셋**: TUM f3 loh, Lip6 Indoor, Bicocca Indoor — 모두 반복 구조의 실내 환경.

#### 프로젝트 적용 시사점

- 상수관로의 **self-similar 구간에서의 false loop closure 문제**에 대한 해법으로 참고 가능
- 다만, 관로 내부는 실내 환경보다 훨씬 더 특징이 부족하므로, 시각 특징 단독 의존 방식의 한계는 여전함
- **비전 + 비시각 센서(음향, 진동, 자기장 등) 결합** 전략이 더 현실적

---

### 2.6 지하 탄광 대규모 매핑을 위한 다중모달 SLAM

| 항목 | 내용 |
|------|------|
| **논문명** | Research on multimodal data enhanced SLAM algorithm for global mapping of underground coal mines |
| **학술지** | Scientific Reports (Nature, 2025) |

#### 핵심 아이디어

지하 탄광 로드웨이의 대규모 매핑을 위해 **LiDAR + 고정밀 관성항법(FOG-IMU, M-SINS) + Wheel Encoder**를 iESKF로 융합하고, **구형 타겟(spherical target)**을 이용한 전역 위치 제약을 부여.

**주요 기여**:
1. **고정밀 FOG-IMU 도입**: MEMS IMU 대신 광섬유 자이로스코프(FOG) 기반 관성항법 시스템을 사용하여 지구 자전각속도까지 감지 가능
2. **구형 타겟 기반 전역 위치 제약**: 터널에 sparse하게 배치된 구형 타겟을 인식하여 절대 위치 제약 및 명확한 loop closure detection 제공
3. **Multi-factor Pose Graph Backend**: LiDAR odometry + 관성 항법 + 구형 타겟 factor를 통합 최적화

**실험**: 500m 규모의 가상 지하 로드웨이 + 실제 탄광 환경에서 LEGO-LOAM, LIO-SAM, FAST-LIO2 대비 우수한 정확도 시연.

#### 프로젝트 적용 시사점

- **관로 내부에 sparse한 식별 마커(타겟)를 설치하는 전략**은 기존 인프라에 최소한의 변경으로 SLAM 정확도를 크게 향상시킬 수 있음
- 마커 기반 절대 위치 제약은 **장거리 관로 탐사 시 누적 drift를 주기적으로 보정**하는 핵심 수단

---

### 2.7 PipeSLAM: 음향 기반 관로 SLAM (관로 전용 선구적 연구)

| 항목 | 내용 |
|------|------|
| **논문명** | PipeSLAM: Simultaneous localisation and mapping in feature sparse water pipes using the Rao-Blackwellised particle filter |
| **저자** | Ke Ma et al. (University of Sheffield) |
| **발표** | 2017 (관로 SLAM 분야의 선구적 연구) |

#### 핵심 아이디어

**비시각적 센서(hydrophone)**를 이용한 관로 전용 SLAM. 시각 특징이 전무한 환경에서의 혁신적 접근.

- **맵 표현**: 하이드로폰으로 금속 관로의 진동 특성을 측정하여 **관로 진동 진폭의 공간적 분포(vibration map)**를 맵으로 구성
- **위치추정**: Terrain-based EKF 및 Particle Filter로 진동 맵 상의 위치를 추정
- **RBPF(Rao-Blackwellised Particle Filter)**: SLAM의 추정 엔진으로 사용

#### 프로젝트 적용 시사점

- **시각이 아닌 음향/진동 기반 맵핑**이라는 발상 자체가 상수관로처럼 시각 정보가 극히 제한된 환경에 매우 적합
- 금속관 뿐 아니라 **초음파 센싱을 활용한 플라스틱 관로 대응** 가능성도 제시
- 최근 연구들이 이 아이디어를 multi-sensor fusion과 결합하여 발전시키는 추세

---

### 2.8 Survey: 상수·하수관로 검사 로봇의 SLAM 종합 리뷰

| 항목 | 내용 |
|------|------|
| **논문명** | Simultaneous Localization and Mapping for Inspection Robots in Water and Sewer Pipe Networks: A Review |
| **저자** | J. M. Aitken et al. (University of Sheffield, University of Leeds 등) |
| **학술지** | IEEE Access, 2021 (Open Access) |
| **관련 Editorial** | Pipeline inspection robots (Frontiers in Robotics and AI, 2024) |

#### 서베이 핵심 정리

이 서베이는 **관로 환경 SLAM의 가장 포괄적인 리뷰**로, 다음 주제를 체계적으로 다룬다:

1. **관로 환경의 SLAM 과제**: 시각 특징 부족, self-similarity, loop closure false positive/negative
2. **수도 산업의 기술적 요구사항**: 손상 위치 정확 보고, 관로 네트워크 매핑, GIS 연동
3. **SLAM 방법론**: EKF, Particle Filter, Graph-based optimization
4. **센서 유형별 검토**: 카메라(visual odometry), IMU, LiDAR, 음향(acoustic), 초음파, 케이블 인코더
5. **맵 표현 방식**: Topological, Metric, Hybrid (관로에 적합한 맵 유형 논의)
6. **미래 과제**: Multi-robot SLAM, 장기 운용 robustness, GIS 사전지식 활용

#### 핵심 통찰

- **관로 환경에서는 단일 센서 SLAM이 근본적으로 한계**가 있으며, **다중 센서 융합이 필수**
- **GIS 데이터 등 사전 지식(prior knowledge)**을 SLAM에 통합하면 초기 맵 추정과 localization 정확도를 크게 향상
- Visual SLAM의 경우 DSO, ORB-SLAM 모두 하수관 이미지에서 제한적 성능을 보임
- **1-DoF 근사 localization**(관로 길이 방향만 추적)이 6-DoF 정밀 odometry보다 더 강건할 수 있다는 관점 제시

---

## 3. 기술 전략 종합 비교

| 전략 | 대표 논문 | 센서 조합 | 장점 | 한계 |
|------|----------|----------|------|------|
| **관로 기하 제약 활용** | VILL-SLAM | Camera+IMU+Laser+LiDAR | 원통 구조를 factor graph에 통합, 장거리 drift 억제 | 센서 패키지 복잡, 비원통 구간(분기, 밸브)에서 제약 무효화 |
| **Wheel Encoder 융합** | Modified FAST-LIO2, Robust-LIWO | LiDAR+IMU+Encoder | 구현 간단, featureless 구간에서 즉각적 성능 향상 | 바퀴 슬립 시 오차 증가, 수중 환경 적용 어려움 |
| **계층적 Fallback** | CompSLAM | Visual+LiDAR+IMU+Leg/Wheel | 센서 퇴화에 강건, 다양한 환경 대응 | 시스템 복잡도 높음, 다중 센서 캘리브레이션 필요 |
| **음향/진동 기반** | PipeSLAM | Hydrophone/Ultrasonic+IMU | 시각 특징 무관, 관로 전용 맵 생성 | 음향 환경 노이즈에 민감, 실시간 처리 과제 |
| **인공 랜드마크** | Coal Mine SLAM | LiDAR+IMU+Encoder+Sphere Targets | 명확한 loop closure, 전역 위치 고정 | 사전 설치 비용, 유지보수 필요 |
| **딥러닝 Loop Closure** | High-Similarity LCD | Camera+IMU | 반복 구조에서 false positive 감소 | 학습 데이터 필요, 계산 비용, 극단적 featureless에선 한계 |

---

## 4. 상수관로 탐사 프로젝트에 대한 제안

검색한 논문들의 분석 결과를 바탕으로, 상수관로 탐사용 자율주행 이동체의 SLAM 시스템 설계를 위해 다음과 같은 단계적 접근을 제안한다.

### 4.1 권장 센서 구성

**최소 구성**: IMU + Wheel Encoder + 1D 거리 센서(관로 벽까지 거리)
**권장 구성**: IMU + Wheel Encoder + 카메라(어안렌즈) + 레이저 프로파일러
**이상적 구성**: 위 + 소형 LiDAR(Realsense L515급) + 음향/진동 센서

### 4.2 알고리즘 전략 (우선순위순)

1. **FAST-LIO2 기반 + Wheel Encoder EKF 융합** — 가장 현실적이고 검증된 접근. 구현 난이도 낮음
2. **관로 기하학적 제약(Cylinder Factor) 추가** — VILL-SLAM의 핵심 아이디어를 factor graph에 통합
3. **Degeneracy Detection** 모듈 구현 — 직선 구간 퇴화 감지 시 encoder 기반 odometry로 자동 전환
4. **인공 마커 기반 Global Constraint** — 관로 내부에 일정 간격으로 RFID/시각 마커 설치하여 절대 위치 보정

### 4.3 맵 표현 전략

- **1-DoF 관로 길이 방향 맵(Pipeline Metric Map)**: 분기점·밸브·엘보 등을 랜드마크로 하는 토폴로지 맵이 관로 환경에 가장 적합
- **3D 점군 맵**: 관로 상태 검사(부식, 변형 등)를 위해 dense 3D reconstruction은 별도 오프라인 처리로 생성

---

## 5. 주요 참고 문헌 목록

1. **Tian, T. et al.** "Visual-Inertial-Laser-Lidar (VILL) SLAM: Real-Time Dense RGB-D Mapping for Pipe Environments." *IROS 2023*. + CMU MS Thesis, 2025.
2. **Robust-LIWO** "Efficient and accurate 3D reconstruction in a featureless tunnel environment." *J. Civil Structural Health Monitoring*, Springer, 2025.
3. **Filip, I. et al.** "LiDAR SLAM with a Wheel Encoder in a Featureless Tunnel Environment." *Electronics* 12(4):1002, MDPI, 2023.
4. **Filip, I. et al.** "Lidar SLAM Comparison in a Featureless Tunnel Environment." *ICCAS 2022*.
5. **CompSLAM** "Complementary Hierarchical Multi-Modal Localization and Mapping for Robot Autonomy in Underground Environments." 2024-2025.
6. **Li, Z. et al.** "Accurate localization of indoor high similarity scenes using visual SLAM combined with loop closure detection algorithm." *PLoS ONE* 19(12), 2024.
7. **Coal Mine SLAM** "Research on multimodal data enhanced SLAM algorithm for global mapping of underground coal mines." *Scientific Reports*, Nature, 2025.
8. **Ma, K. et al.** "PipeSLAM: Simultaneous localisation and mapping in feature sparse water pipes using the RBPF." 2017.
9. **Aitken, J. M. et al.** "Simultaneous Localization and Mapping for Inspection Robots in Water and Sewer Pipe Networks: A Review." *IEEE Access*, 2021.
10. **Mihaylova, L. et al.** "Editorial: Pipeline inspection robots." *Frontiers in Robotics and AI* 11:1497809, 2024.
11. **LVI-Fusion** "A Robust Lidar-Visual-Inertial SLAM Scheme." *Remote Sensing* 16(9):1524, MDPI, 2024.

---

*본 보고서는 Semantic Scholar, arXiv, Web 검색을 통해 수집한 논문들을 기반으로 작성되었습니다.*
