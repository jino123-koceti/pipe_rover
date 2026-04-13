import open3d as o3d
import numpy as np
import cv2

def cylindrical_texture_map(pcd, rgb_image):
    points = np.asarray(pcd.points)
    colors = np.zeros_like(points)

    x, y, z = points[:,0], points[:,1], points[:,2]
    theta = (np.arctan2(y, x) + 2 * np.pi) % (2 * np.pi)
    height = z

    # 정규화
    theta_bins = 1024
    z_bins = 512
    theta_idx = (theta / (2*np.pi) * (theta_bins-1)).astype(int)
    z_min, z_max = np.min(height), np.max(height)
    z_idx = ((height - z_min) / (z_max - z_min) * (z_bins-1)).astype(int)

    # RGB 정합이 없는 경우, 가까운 시야 중심 기준으로 매핑
    # 예시로 RGB 중앙 픽셀 기준 컬러 할당 (임시)
    img_center = rgb_image[rgb_image.shape[0]//2, rgb_image.shape[1]//2]
    tex_map = np.ones((z_bins, theta_bins, 3), dtype=np.uint8) * img_center

    for i in range(len(points)):
        tex_map[z_idx[i], theta_idx[i]] = img_center  # TODO: 실제 픽셀 정합 필요

    cv2.imwrite("unwrapped_texture.png", tex_map)
