import cv2
import os

def extract_frames(video_path, output_dir, interval_sec=6):
    # 디렉토리 생성
    os.makedirs(output_dir, exist_ok=True)

    # 영상 열기
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration_sec = total_frames / fps

    print(f"🎞 FPS: {fps}, 총 프레임: {total_frames}, 영상 길이: {duration_sec:.2f}초")

    count = 0
    for sec in range(0, int(duration_sec), interval_sec):
        frame_idx = int(sec * fps)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = cap.read()

        if ret:
            filename = os.path.join(output_dir, f"frame2_{count:03d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"✅ Saved: {filename}")
            count += 1
        else:
            print(f"❌ Failed to read frame at {sec}s")

    cap.release()
    print("📁 모든 프레임 저장 완료!")

# 실행
if __name__ == "__main__":
    extract_frames(
        video_path="video/250724/output_video3.mp4",
        output_dir="img",
        interval_sec=6
    )
