import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def filter_pipe_roi(pcd_path, x_range, y_range, z_range):
    # PCD 파일 로드
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    # XYZ 범위 마스크
    mask = (
        (points[:, 0] > x_range[0]) & (points[:, 0] < x_range[1]) &
        (points[:, 1] > y_range[0]) & (points[:, 1] < y_range[1]) &
        (points[:, 2] > z_range[0]) & (points[:, 2] < z_range[1])
    )

    # 필터링 적용
    filtered_pcd = pcd.select_by_index(np.where(mask)[0])
    o3d.io.write_point_cloud("filtered_pipe_roi.pcd", filtered_pcd)
    print("✅ 필터링 완료: filtered_pipe_roi.pcd 저장됨")

    # 시각화
    o3d.visualization.draw_geometries([filtered_pcd])

    # XY 평면 상의 top view 플롯
    filtered_points = np.asarray(filtered_pcd.points)
    plt.figure(figsize=(10, 6))
    plt.scatter(filtered_points[:, 0], filtered_points[:, 1], s=0.5, c='black')
    plt.title("Top View of Filtered Pipe (X-Y Plane)")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


# 실행
filter_pipe_roi(
    pcd_path="pcd/250724/scans2.pcd",
    x_range=(-0.45, 9.18),   # 관 길이 범위
    y_range=(-2, 3),    # 관 폭 (직경 1.5m 기준)
    z_range=(-0.6, 2.5)  # 관 높이 범위
)

def unwrap_cylinder(pcd_path, radius_center=(5, 1.0), resolution=(720, 1000)):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    x0, y0 = radius_center
    dx = points[:, 0] - x0
    dy = points[:, 1] - y0
    theta = np.arctan2(dy, dx)
    theta_deg = np.degrees(theta) % 360
    z = points[:, 2]

    # z 범위 체크
    z_min, z_max = z.min(), z.max()
    print(f"z range: {z_min:.2f} ~ {z_max:.2f}")

    theta_res, z_res = resolution
    theta_idx = (theta_deg / 360 * (theta_res - 1)).astype(int)
    z_norm = (z - z_min) / (z_max - z_min)
    z_idx = (z_norm * (z_res - 1)).astype(int)

    # 이미지 생성 (초기값 255 = 흰색)
    img = np.ones((z_res, theta_res), dtype=np.uint8) * 255
    img[z_idx, theta_idx] = 0

    # 시각화
    plt.figure(figsize=(12, 4))
    plt.imshow(img, cmap='gray', aspect='auto', origin='lower')
    plt.title("Unwrapped Pipe Surface (θ vs z)")
    plt.xlabel("θ (angle)")
    plt.ylabel("z (height)")
    plt.tight_layout()
    plt.show()

# 실행
unwrap_cylinder("filtered_pipe_roi.pcd", radius_center=(4.8, 1.0), resolution=(720, 1000))

def unwrap_cylinder_theta_x(pcd_path, radius_center=(5, 1.0), resolution=(720, 1000)):
    import open3d as o3d
    import numpy as np
    import matplotlib.pyplot as plt

    # 포인트 클라우드 로드
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    # 중심 기준 θ 계산
    x0, y0 = radius_center
    dx = points[:, 0] - x0
    dy = points[:, 1] - y0
    theta = np.arctan2(dy, dx)
    theta_deg = np.degrees(theta) % 360
    x = points[:, 0]

    # 디버깅용 출력
    print(f"x range: {x.min():.2f} ~ {x.max():.2f}")
    print(f"θ range: {theta_deg.min():.2f} ~ {theta_deg.max():.2f}")

    # θ-x 평면으로 맵핑
    theta_res, x_res = resolution
    theta_idx = np.clip((theta_deg / 360 * (theta_res - 1)).astype(int), 0, theta_res - 1)
    x_norm = (x - x.min()) / (x.max() - x.min() + 1e-8)
    x_idx = np.clip((x_norm * (x_res - 1)).astype(int), 0, x_res - 1)

    # 이미지 생성
    img = np.ones((x_res, theta_res), dtype=np.uint8) * 255
    img[x_idx, theta_idx] = 0

    # 시각화
    plt.figure(figsize=(12, 4))
    plt.imshow(img, cmap='gray', aspect='auto', origin='lower')
    plt.title("Unwrapped Pipe Surface (θ vs x)")
    plt.xlabel("θ (angle)")
    plt.ylabel("x (length along pipe)")
    plt.tight_layout()
    plt.show()


# 실행
unwrap_cylinder_theta_x("filtered_pipe_roi.pcd", radius_center=(4.8, 1.0), resolution=(720, 1000))
