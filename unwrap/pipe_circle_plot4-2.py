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
writer = None

# === 5. 이전 중심점 저장 변수 ===
prev_big = None
prev_small = None

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

    cx_big = cy_big = r_big = None
    cx_small = cy_small = r_small = None

    if boxes is not None and len(boxes) > 0:
        class_ids = boxes.cls.cpu().numpy()
        confidences = boxes.conf.cpu().numpy()
        xyxy = boxes.xyxy.cpu().numpy()

        def get_best_circle(class_id):
            indices = np.where(class_ids == class_id)[0]
            if len(indices) == 0:
                return None
            best_idx = indices[np.argmax(confidences[indices])]
            x1, y1, x2, y2 = xyxy[best_idx]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            r = int(min(x2 - x1, y2 - y1) / 2)
            return (cx, cy, r)

        big = get_best_circle(0)  # circle1
        small = get_best_circle(1)  # circle2

        if big:
            prev_big = big
        if small:
            prev_small = small

    # === 이전 값으로 fallback ===
    if prev_big is None or prev_small is None:
        print(f"⚠️ frame {frame_idx}: circle1 or circle2 못 찾음 → 이전 프레임 사용")
    if prev_big is None or prev_small is None:
        print(f"⛔ frame {frame_idx}: 시작부터 원 2개 미탐지 → 스킵")
        continue

    cx_big, cy_big, r_big = prev_big
    cx_small, cy_small, r_small = prev_small

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

    # === 원본에 원/중심점 표시 ===
    img_with_circles = frame.copy()
    cv2.circle(img_with_circles, (cx_big, cy_big), r_big, (0, 255, 0), 2)
    cv2.circle(img_with_circles, (cx_big, cy_big), 5, (0, 0, 255), -1)
    cv2.putText(img_with_circles, "circle1", (cx_big + 10, cy_big), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.circle(img_with_circles, (cx_small, cy_small), r_small, (0, 255, 255), 2)
    cv2.circle(img_with_circles, (cx_small, cy_small), 5, (255, 0, 0), -1)
    cv2.putText(img_with_circles, "circle2", (cx_small + 10, cy_small), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # === 크기 맞춰서 상하 병합 ===
    unwrap_frame_resized = cv2.resize(unwrap_frame, (img_with_circles.shape[1], img_with_circles.shape[0]))
    combined_frame = np.vstack((img_with_circles, unwrap_frame_resized))

    # === VideoWriter 초기화 ===
    if not unwrap_size_fixed:
        combined_H, combined_W = combined_frame.shape[:2]
        writer = cv2.VideoWriter(SAVE_PATH, cv2.VideoWriter_fourcc(*'mp4v'), fps, (combined_W, combined_H))
        unwrap_size_fixed = True

    # === 저장 ===
    writer.write(combined_frame)

# 마무리
cap.release()
if writer is not None:
    writer.release()
print("✅ 언랩된 영상 저장 완료:", SAVE_PATH)
