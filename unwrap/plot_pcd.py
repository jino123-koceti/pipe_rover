import open3d as o3d

def plot_pcd(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    print(f"✅ Loaded point cloud: {pcd}")
    o3d.visualization.draw_geometries([pcd],
                                      zoom=0.5,
                                      front=[0.0, 0.0, -1.0],
                                      lookat=[0.0, 0.0, 0.0],
                                      up=[0.0, -1.0, 0.0])

if __name__ == "__main__":
    plot_pcd("pcd/250724/scans2.pcd")
