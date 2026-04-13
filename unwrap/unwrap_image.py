import cv2
import numpy as np
import matplotlib.pyplot as plt

def unwrap_conical_image(image_path, center=None, radius=None, output_shape=(720, 1000)):
    img = cv2.imread(image_path)
    h, w = img.shape[:2]

    if center is None:
        center = (w // 2, h // 2)
    if radius is None:
        radius = min(center[0], center[1], w - center[0], h - center[1])

    # 언랩 영역 (출력은 θ x r)
    unwrapped = cv2.warpPolar(
        img,
        dsize=output_shape[::-1],  # (cols, rows)
        center=center,
        maxRadius=radius,
        flags=cv2.WARP_POLAR_LINEAR + cv2.WARP_FILL_OUTLIERS
    )

    # θ축을 X축으로 (수평으로 펼치기 위해)
    unwrapped = cv2.rotate(unwrapped, cv2.ROTATE_90_CLOCKWISE)

    # 시각화
    plt.figure(figsize=(14, 6))
    plt.subplot(1, 2, 1)
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title("Original Image")
    plt.axis("off")

    plt.subplot(1, 2, 2)
    plt.imshow(cv2.cvtColor(unwrapped, cv2.COLOR_BGR2RGB))
    plt.title("Unwrapped Image (θ → x, r → y)")
    plt.axis("off")
    plt.tight_layout()
    plt.show()

# 실행
unwrap_conical_image(
    image_path="video/250724/test.jpg",
    center=(1018, 616),    # 관 중심 수동 설정 (예: 이미지 정중앙)
    radius=1018,           # 관 반경 추정
    output_shape=(1440, 720)  # (r, θ)
)
