## circle1 label 기준으로 펼침

import os
import cv2
import numpy as np
from ultralytics import YOLO

# 1. 경로 설정
IMG_PATH = "/home/koceti/unwrap_ws/data/pipe_learning_data/test2/5.jpg"
BEST_MODEL = "pipe_training/yolov8m_circle_detect/weights/best.pt"

# 2. 모델 로드 및 이미지 추론
model = YOLO(BEST_MODEL)
results = model(IMG_PATH)[0]
img = cv2.imread(IMG_PATH)
H, W = img.shape[:2]

# 3. 중심점 및 반지름 추출 (circle1 → circle2 fallback)
boxes = results.boxes
cx, cy, radius = None, None, None

if boxes is not None and len(boxes) > 0:
    class_ids = boxes.cls.cpu().numpy()
    confidences = boxes.conf.cpu().numpy()
    
    # 우선순위: circle1 (label 0)
    def select_best_bbox_by_class(target_class):
        indices = np.where(class_ids == target_class)[0]
        if len(indices) == 0:
            return None
        best_idx = indices[np.argmax(confidences[indices])]
        return best_idx

    best_idx = select_best_bbox_by_class(0)  # circle1
    label_used = "circle1"

    if best_idx is None:
        best_idx = select_best_bbox_by_class(1)  # circle2 fallback
        label_used = "circle2"

    if best_idx is None:
        raise ValueError("❌ circle1, circle2 클래스 모두 탐지 실패")

    box = boxes.xyxy[best_idx].tolist()
    x1, y1, x2, y2 = map(int, box)
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)
    radius = int(min((x2 - x1), (y2 - y1)) / 2)

    print(f"✅ {label_used} 기준 중심점: ({cx}, {cy}), 반지름: {radius}")
else:
    raise ValueError("❌ 원형 객체 탐지 실패")


# 4. 원본 이미지에 중심점과 원 표시
img_with_circle = img.copy()
cv2.circle(img_with_circle, (cx, cy), 5, (0, 0, 255), -1)              # 중심점 (빨강 점)
cv2.circle(img_with_circle, (cx, cy), radius, (0, 255, 0), 2)          # 예측된 원 (녹색)

# 5. 반지름 범위 설정 (전체 이미지 커버하도록)
corners = [(0, 0), (0, H), (W, 0), (W, H)]
r_max = max([np.sqrt((cx - x)**2 + (cy - y)**2) for x, y in corners])
r_min = 0
angle_res = 720
radius_res = int(r_max - r_min)

# 6. 매핑 계산
theta = np.linspace(0, 2 * np.pi, angle_res)
radius_vals = np.linspace(r_min, r_max, radius_res)
theta_grid, radius_grid = np.meshgrid(theta, radius_vals)

x_map = cx + (radius_grid * np.cos(theta_grid)).astype(np.float32)
y_map = cy + (radius_grid * np.sin(theta_grid)).astype(np.float32)

# 7. 리매핑
unwrap_img = cv2.remap(img, x_map, y_map, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
unwrap_img = cv2.flip(unwrap_img, 1)

# 8. 결과 시각화
cv2.imshow("Original", img)
cv2.imshow("Unwrapped Pipe (Full)", unwrap_img)
cv2.imshow("Detected Circle", img_with_circle)
cv2.waitKey(0)
cv2.destroyAllWindows()


import matplotlib.pyplot as plt

# === 9. 이미지 4분할 ===
q1 = img[0:cy, cx:W]      # 우상
q2 = img[0:cy, 0:cx]      # 좌상
q3 = img[cy:H, 0:cx]      # 좌하
q4 = img[cy:H, cx:W]      # 우하

# === 10. 2x2 서브플롯으로 시각화 ===
fig, axs = plt.subplots(2, 2, figsize=(10, 8))
axs[0, 0].imshow(cv2.cvtColor(q2, cv2.COLOR_BGR2RGB))
axs[0, 0].set_title("1: Top-Left")
axs[0, 1].imshow(cv2.cvtColor(q1, cv2.COLOR_BGR2RGB))
axs[0, 1].set_title("2: Top-Right")
axs[1, 0].imshow(cv2.cvtColor(q3, cv2.COLOR_BGR2RGB))
axs[1, 0].set_title("3: Bottom-Left")
axs[1, 1].imshow(cv2.cvtColor(q4, cv2.COLOR_BGR2RGB))
axs[1, 1].set_title("4: Bottom-Right")

for ax in axs.flat:
    ax.axis("off")

plt.tight_layout()
plt.show()
