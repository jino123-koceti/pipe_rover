import cv2
import numpy as np
import os

def cylindrical_unwrap(img, radius=None, output_height=None, output_width=None):
    h, w = img.shape[:2]

    # 반지름, 출력 영상 크기 기본값 설정
    radius = radius or w // 2
    output_height = output_height or h
    output_width = output_width or int(2 * np.pi * radius)

    # 출력 프레임 생성
    unwrapped = np.zeros((output_height, output_width, 3), dtype=img.dtype)

    for y in range(output_height):
        for x in range(output_width):
            theta = 2 * np.pi * x / output_width
            xi = int(w / 2 + radius * np.sin(theta))
            yi = y
            if 0 <= xi < w and 0 <= yi < h:
                unwrapped[y, x] = img[yi, xi]

    return unwrapped

def improved_cylindrical_unwrap(img, output_height=None, output_width=None):
    h, w = img.shape[:2]

    # 기본 중심은 영상의 중심
    center_x = w // 2
    center_y = h // 2
    radius = min(center_x, center_y)

    output_height = output_height or radius
    output_width = output_width or 2 * int(np.pi * radius)

    # 각도 (theta)와 반지름 (r) 생성
    theta_range = np.linspace(0, 2 * np.pi, output_width)
    radius_range = np.linspace(0, radius, output_height)

    # 좌표 매핑 생성
    map_x = np.zeros((output_height, output_width), dtype=np.float32)
    map_y = np.zeros((output_height, output_width), dtype=np.float32)

    for i in range(output_height):
        for j in range(output_width):
            r = radius_range[i]
            theta = theta_range[j]

            x = center_x + r * np.cos(theta)
            y = center_y + r * np.sin(theta)

            map_x[i, j] = x
            map_y[i, j] = y

    unwrapped = cv2.remap(img, map_x, map_y, interpolation=cv2.INTER_LINEAR)
    return unwrapped



def process_video(input_path, output_path):
    cap = cv2.VideoCapture(input_path)

    if not cap.isOpened():
        print(f"❌ Failed to open video: {input_path}")
        return

    # 첫 프레임으로 크기 및 FPS 확인
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to read first frame")
        cap.release()
        return

    unwrap_frame = improved_cylindrical_unwrap(frame)
    out_h, out_w = unwrap_frame.shape[:2]

    # 단일 채널일 경우 BGR로 변환
    if len(unwrap_frame.shape) == 2:
        unwrap_frame = cv2.cvtColor(unwrap_frame, cv2.COLOR_GRAY2BGR)

    # FPS 및 VideoWriter 설정
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0.0:
        fps = 30.0

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_path, fourcc, fps, (out_w, out_h))

    if not out.isOpened():
        print("❌ VideoWriter failed to open.")
        return

    # 첫 프레임 저장
    out.write(unwrap_frame)

    # 이후 프레임 변환 반복
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        unwrap_frame = improved_cylindrical_unwrap(frame)

        # 동일한 크기 보장
        if unwrap_frame.shape[:2] != (out_h, out_w):
            unwrap_frame = cv2.resize(unwrap_frame, (out_w, out_h))

        out.write(unwrap_frame)


    cap.release()
    out.release()
    print(f"✅ Saved unwrapped video to {output_path}")

if __name__ == "__main__":
    input_video = "video/250724/output_video2.mp4"
    output_video = "unwrapped_output_video22.avi"

    process_video(input_video, output_video)
