import cv2
import numpy as np
import os

def auto_label_yolo(img_path, label_dir, class_id=0):
    img = cv2.imread(img_path)
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.medianBlur(gray, 5)

    circles = cv2.HoughCircles(
        gray_blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
        param1=150, param2=50, minRadius=30, maxRadius=200
    )

    label_path = os.path.join(label_dir, os.path.splitext(os.path.basename(img_path))[0] + '.txt')
    os.makedirs(label_dir, exist_ok=True)

    if circles is not None:
        circles = np.uint16(np.around(circles[0, :]))
        with open(label_path, 'w') as f:
            for x, y, r in circles:
                # Normalize for YOLO format
                x_center = x / w
                y_center = y / h
                width = height = (2 * r) / w  # assuming square box
                f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
    else:
        print(f"[!] No circles found in {img_path}")

image_dir = "img"
label_dir = "labels"

for fname in sorted(os.listdir(image_dir)):
    if fname.endswith(".jpg"):
        auto_label_yolo(os.path.join(image_dir, fname), label_dir)
