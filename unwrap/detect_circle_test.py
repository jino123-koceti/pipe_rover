import cv2
import numpy as np
import matplotlib.pyplot as plt

# 이미지 경로
img_path = 'img/frame1_009.jpg'

# 이미지 불러오기
img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"이미지를 찾을 수 없습니다: {img_path}")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray_blur = cv2.medianBlur(gray, 5)

# Hough 원 검출
circles = cv2.HoughCircles(
    gray_blur,
    cv2.HOUGH_GRADIENT,
    dp=1.2,
    minDist=100,
    param1=200,
    param2=50,
    minRadius=30,
    maxRadius=200
)

# 원 시각화
if circles is not None:
    circles = np.uint16(np.around(circles))
    for x, y, r in circles[0, :]:
        cv2.circle(img, (x, y), r, (0, 0, 255), 2)
        cv2.circle(img, (x, y), 2, (0, 255, 0), 3)
    print(f"✅ 원 {len(circles[0])}개 검출됨")
else:
    print("❌ 원 검출 실패")

# 출력
plt.figure(figsize=(10, 6))
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.title("Detected Circle (frame1_009.jpg)")
plt.axis('off')
plt.show()
