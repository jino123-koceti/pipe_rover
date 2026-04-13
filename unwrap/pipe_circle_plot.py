## Yolo 중심점 식별 모델 성능 테스트 

import os
import cv2
from ultralytics import YOLO

# 경로 설정
BASE_DIR = "/home/koceti/unwrap_ws/data/pipe_learning_data"
TEST_DIR = os.path.join(BASE_DIR, "test2")
RESULT_DIR = "pipe_results"
BEST_MODEL = "pipe_training/yolov8m_circle_detect/weights/best.pt"

os.makedirs(RESULT_DIR, exist_ok=True)

# 모델 불러오기
model = YOLO(BEST_MODEL)

# 테스트 이미지 추론 및 시각화
image_files = [f for f in os.listdir(TEST_DIR) if f.lower().endswith((".jpg", ".png"))]

for img_file in image_files:
    img_path = os.path.join(TEST_DIR, img_file)
    results = model(img_path)[0]  # 첫 결과만 사용
    img = cv2.imread(img_path)

    boxes = results.boxes
    if boxes is not None:
        for i, box in enumerate(boxes.xyxy):
            x1, y1, x2, y2 = map(int, box.tolist())
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            radius = int(min((x2 - x1), (y2 - y1)) / 2)

            # 클래스 및 신뢰도
            cls_id = int(boxes.cls[i].item())
            conf = boxes.conf[i].item()
            class_name = model.names[cls_id]

            # 원과 중심점 그리기
            cv2.circle(img, (cx, cy), radius, (255, 0, 0), 2)  # 파란 원
            cv2.circle(img, (cx, cy), 3, (0, 0, 255), -1)      # 빨간 중심점
            cv2.putText(img, f"{class_name} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 저장
    save_path = os.path.join(RESULT_DIR, f"circle_{img_file}")
    cv2.imwrite(save_path, img)
    print(f"✅ {img_file} → 원+중심점 시각화 저장됨: {save_path}")
