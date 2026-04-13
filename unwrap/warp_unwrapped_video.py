import numpy as np
import cv2

def load_radius_curve(path="radius_by_theta_filtered.npy"):
    r_curve = np.load(path)
    r_curve = np.clip(r_curve, 0.01, np.percentile(r_curve, 95))  # 노이즈 제거
    r_curve /= np.max(r_curve)  # 0~1 정규화
    return r_curve

def warp_frame(frame, r_curve):
    h, w = frame.shape[:2]

    map_x = np.zeros((h, w), dtype=np.float32)
    map_y = np.tile(np.arange(h).reshape(h, 1), (1, w)).astype(np.float32)

    for x in range(w):
        scale = r_curve[int(x * len(r_curve) / w)]
        map_x[:, x] = x * scale  # 수평 좌표 보정

    # 범위 초과 픽셀 잘라냄
    max_x = int(np.max(map_x))
    map_x = np.clip(map_x, 0, w - 1)

    warped = cv2.remap(frame, map_x, map_y, interpolation=cv2.INTER_LINEAR)
    return warped[:, :w]

def process_video(input_path, output_path, r_curve):
    cap = cv2.VideoCapture(input_path)

    fps = cap.get(cv2.CAP_PROP_FPS)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        warped = warp_frame(frame, r_curve)
        out.write(warped)

    cap.release()
    out.release()
    print(f"✅ Saved warped video to {output_path}")

if __name__ == "__main__":
    r_curve = load_radius_curve("radius_by_theta_filtered.npy")
    process_video("unwrapped_output_video2.mp4", "warped_unwrapped_video2.mp4", r_curve)
