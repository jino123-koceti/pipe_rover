## YOLO 기반 원 2개 인식 해서 펼침 (영상기반)

import os
import cv2
import numpy as np
from ultralytics import YOLO

# === 1. 경로 설정 ===
VIDEO_PATH = "/home/koceti/unwrap_ws/data/pipe_learning_data/videos/output_video.mp4"
BEST_MODEL = "pipe_training/yolov8m_circle_detect/weights/best.pt"
SAVE_PATH = "unwrapped_output.mp4"

# === 2. 모델 로드 ===
model = YOLO(BEST_MODEL)

# === 3. 비디오 설정 ===
cap = cv2.VideoCapture(VIDEO_PATH)
fps = cap.get(cv2.CAP_PROP_FPS)
W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
angle_res = 720

# === 4. 저장기 초기화 ===
unwrap_size_fixed = False
unwrap_W, unwrap_H = 0, 0
writer = None

frame_idx = 0
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    frame_idx += 1
    print(f"📽 frame {frame_idx}")

    # === YOLO 추론 ===
    results = model(frame)[0]
    boxes = results.boxes

    if boxes is None or len(boxes) < 2:
        print(f"⚠️ Skipping frame {frame_idx} (원 2개 미만)")
        continue

    # === 큰 원과 작은 원 선택 ===
    areas = (boxes.xyxy[:, 2] - boxes.xyxy[:, 0]) * (boxes.xyxy[:, 3] - boxes.xyxy[:, 1])
    sorted_idx = np.argsort(areas.cpu().numpy())[::-1]

    # 큰 원
    x1b, y1b, x2b, y2b = map(int, boxes.xyxy[sorted_idx[0]].tolist())
    cx_big = int((x1b + x2b) / 2)
    cy_big = int((y1b + y2b) / 2)
    r_big = int(min(x2b - x1b, y2b - y1b) / 2)

    # 작은 원
    x1s, y1s, x2s, y2s = map(int, boxes.xyxy[sorted_idx[1]].tolist())
    cx_small = int((x1s + x2s) / 2)
    cy_small = int((y1s + y2s) / 2)
    r_small = int(min(x2s - x1s, y2s - y1s) / 2)

    # === unwrap 범위 계산 ===
    corners = [(0, 0), (0, H), (W, 0), (W, H)]
    r_max = max([np.sqrt((cx_big - x)**2 + (cy_big - y)**2) for x, y in corners])
    r_min = 0
    radius_res = int(r_max - r_min)

    theta = np.linspace(0, 2 * np.pi, angle_res)
    radius_vals = np.linspace(r_min, r_max, radius_res)
    theta_grid, radius_grid = np.meshgrid(theta, radius_vals)

    r_switch = r_big
    delta = 20

    x_map = np.zeros_like(theta_grid, dtype=np.float32)
    y_map = np.zeros_like(theta_grid, dtype=np.float32)

    for i in range(radius_res):
        r = radius_vals[i]

        if r < r_switch - delta:
            cx, cy = cx_big, cy_big
        elif r > r_switch + delta:
            cx, cy = cx_small, cy_small
        else:
            alpha = (r - (r_switch - delta)) / (2 * delta)
            cx = int((1 - alpha) * cx_big + alpha * cx_small)
            cy = int((1 - alpha) * cy_big + alpha * cy_small)

        x_map[i] = cx + r * np.cos(theta)
        y_map[i] = cy + r * np.sin(theta)

    # === 리매핑 ===
    unwrap_frame = cv2.remap(frame, x_map, y_map, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    unwrap_frame = cv2.flip(unwrap_frame, 1)

    # === VideoWriter 초기화 ===
    if not unwrap_size_fixed:
        unwrap_H, unwrap_W = unwrap_frame.shape[:2]
        writer = cv2.VideoWriter(SAVE_PATH, cv2.VideoWriter_fourcc(*'mp4v'), fps, (unwrap_W, unwrap_H))
        unwrap_size_fixed = True

    # === 해상도 확인 & 맞춤 ===
    h_now, w_now = unwrap_frame.shape[:2]
    if (h_now, w_now) != (unwrap_H, unwrap_W):
        unwrap_frame = cv2.resize(unwrap_frame, (unwrap_W, unwrap_H))
        print(f"⚠️ Resized frame {frame_idx} from ({w_now}, {h_now}) to ({unwrap_W}, {unwrap_H})")

    writer.write(unwrap_frame)

# 마무리
cap.release()
if writer is not None:
    writer.release()
print("✅ 언랩된 영상 저장 완료:", SAVE_PATH)
