import os
import matplotlib.pyplot as plt
from ultralytics import YOLO
from PIL import Image
import cv2

# 1. 경로 설정
BASE_DIR = "/home/koceti/unwrap_ws/data/pipe_learning_data"
TEST_DIR = os.path.join(BASE_DIR, "test2")
RESULT_DIR = "pipe_results"
DATA_YAML = os.path.join(BASE_DIR, "pipe_data.yaml")
BEST_MODEL= "pipe_training/yolov8m_circle_detect/weights/best.pt"

# 2. data.yaml 파일 생성
data_yaml_content = f"""
train: {os.path.join(BASE_DIR, "images")}
val: {os.path.join(BASE_DIR, "images")}
nc: 2
names: ["circle1", "circle2"]
"""
os.makedirs(RESULT_DIR, exist_ok=True)
with open(DATA_YAML, "w") as f:
    f.write(data_yaml_content)

# 3. 모델 학습(재학습)
model = YOLO("yolov8m.pt")
train_results = model.train(
    data=DATA_YAML,
    epochs=500,
    imgsz=640,
    project="pipe_training",
    name="yolov8m_circle_detect",
    exist_ok=True
)

# 3. 모델 불러오기
#model = YOLO(BEST_MODEL)


# 4. 학습 결과 시각화
metrics = train_results.results_dict  # 🔁 여기만 수정

# 그래프 그리기
plt.figure(figsize=(10, 6))
plt.bar(["Precision", "Recall", "mAP50", "mAP50-95"], [
    metrics["metrics/precision(B)"],
    metrics["metrics/recall(B)"],
    metrics["metrics/mAP50(B)"],
    metrics["metrics/mAP50-95(B)"]
])
plt.title("Validation Performance (Best.pt)")
plt.ylim(0, 1.05)
plt.ylabel("Score")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(RESULT_DIR, "validation_metrics_bar.png"))
plt.close()

# 4. 테스트 이미지에 대해 추론
#image_files = [f for f in os.listdir(TEST_DIR) if f.lower().endswith((".jpg", ".png"))]

#for img_file in image_files:
#    img_path = os.path.join(TEST_DIR, img_file)
#    results = model(img_path)

    # 결과 이미지 시각화 및 저장
#    result_img = results[0].plot()
#    save_path = os.path.join(RESULT_DIR, f"pred_{img_file}")
#    cv2.imwrite(save_path, result_img)
#    print(f"✅ {img_file} → 저장됨: {save_path}")
