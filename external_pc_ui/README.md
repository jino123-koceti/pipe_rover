# 외부 PC 모니터링 UI (Zenoh)

로봇(Jetson)에서 `zenoh_client`가 실행 중일 때, **ROS 2 없이** 외부 PC에서 SLAM 상태를 모니터링합니다.

## 요구 사항

- Python 3.10+
- 그래픽 환경 (OpenGL 3D 뷰)

## 설치

```bash
cd external_pc_ui
pip install -r requirements.txt
```

## 실행

```bash
python3 main.py
```

앱에서 **로봇 IP**와 **포트**(기본 7447)를 입력한 뒤 **연결**을 누릅니다.

## 프로토콜

토픽·msgpack 포맷은 저장소 루트의 [`docs/ZENOH_UI_DEVELOPMENT_GUIDE.md`](../docs/ZENOH_UI_DEVELOPMENT_GUIDE.md)를 참고하세요.

## 파일

| 파일 | 설명 |
|------|------|
| `main.py` | PyQt5 메인 창 (3D, 카메라, 상태, 명령) |
| `zenoh_receiver.py` | Zenoh 구독 → Qt 시그널 |
| `requirements.txt` | pip 의존성 |
