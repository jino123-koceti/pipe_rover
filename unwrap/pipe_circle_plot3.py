## 2개 원 식별해서 구간 별로 중심점을 변경해서 펼침

import os
import cv2
import numpy as np
from ultralytics import YOLO

# 1. 경로 설정
IMG_PATH = "/home/koceti/unwrap_ws/data/pipe_learning_data/test2/4.jpg"
BEST_MODEL = "pipe_training/yolov8m_circle_detect/weights/best.pt"

# 2. 모델 로드 및 이미지 추론
model = YOLO(BEST_MODEL)
results = model(IMG_PATH)[0]
img = cv2.imread(IMG_PATH)
H, W = img.shape[:2]

# 3. 모든 원 중심, 반지름 추출
boxes = results.boxes
if boxes is None or len(boxes) < 2:
    raise ValueError("❌ 2개 이상의 원형 객체가 필요합니다.")

# 넓이 기준으로 큰 원과 작은 원 선택
areas = (boxes.xyxy[:, 2] - boxes.xyxy[:, 0]) * (boxes.xyxy[:, 3] - boxes.xyxy[:, 1])
sorted_idx = np.argsort(areas.cpu().numpy())[::-1]  # 내림차순

# 큰 원
big_idx = sorted_idx[0]
x1b, y1b, x2b, y2b = map(int, boxes.xyxy[big_idx].tolist())
cx_big = int((x1b + x2b) / 2)
cy_big = int((y1b + y2b) / 2)
r_big = int(min(x2b - x1b, y2b - y1b) / 2)

# 작은 원
small_idx = sorted_idx[1]
x1s, y1s, x2s, y2s = map(int, boxes.xyxy[small_idx].tolist())
cx_small = int((x1s + x2s) / 2)
cy_small = int((y1s + y2s) / 2)
r_small = int(min(x2s - x1s, y2s - y1s) / 2)

print(f"✅ 큰 원 중심: ({cx_big}, {cy_big}), 반지름: {r_big}")
print(f"✅ 작은 원 중심: ({cx_small}, {cy_small}), 반지름: {r_small}")

# 4. 시각화용 이미지 생성
img_with_circle = img.copy()
cv2.circle(img_with_circle, (cx_big, cy_big), r_big, (255, 0, 0), 2)
cv2.circle(img_with_circle, (cx_small, cy_small), r_small, (0, 255, 255), 2)
cv2.circle(img_with_circle, (cx_big, cy_big), 5, (0, 0, 255), -1)
cv2.circle(img_with_circle, (cx_small, cy_small), 5, (0, 255, 0), -1)

# 5. 전체 영역 펼치기 설정
corners = [(0, 0), (0, H), (W, 0), (W, H)]
r_max = max([np.sqrt((cx_big - x)**2 + (cy_big - y)**2) for x, y in corners])
r_min = 0
angle_res = 720
radius_res = int(r_max - r_min)

# 6. 매핑 계산 (dual-center unwrap)
theta = np.linspace(0, 2 * np.pi, angle_res)
radius_vals = np.linspace(r_min, r_max, radius_res)
theta_grid, radius_grid = np.meshgrid(theta, radius_vals)

# 전환 지점 설정
r_switch = r_big
delta = 20  # 부드러운 보간 구간 길이

x_map = np.zeros_like(theta_grid, dtype=np.float32)
y_map = np.zeros_like(theta_grid, dtype=np.float32)

for i in range(radius_res):
    r = radius_vals[i]

    if r < r_switch - delta:
        cx, cy = cx_big, cy_big
    elif r > r_switch + delta:
        cx, cy = cx_small, cy_small
    else:
        # 보간 전환
        alpha = (r - (r_switch - delta)) / (2 * delta)
        cx = int((1 - alpha) * cx_big + alpha * cx_small)
        cy = int((1 - alpha) * cy_big + alpha * cy_small)

    x_map[i] = cx + r * np.cos(theta)
    y_map[i] = cy + r * np.sin(theta)

# 7. 리매핑
unwrap_img = cv2.remap(img, x_map, y_map, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
unwrap_img = cv2.flip(unwrap_img, 1)

# 8. 결과 시각화
cv2.imshow("Original", img)
cv2.imshow("Detected Circles", img_with_circle)
cv2.imshow("Dual-Center Unwrapped Pipe", unwrap_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
