import open3d as o3d

def check_pcd(path):
    print(f"🔍 Checking: {path}")
    pcd = o3d.io.read_point_cloud(path)
    print(f"📌 Point Cloud Info:\n  - Points: {len(pcd.points)}")
    print(f"  - Has color: {pcd.has_colors()}")
    print(f"  - Has normals: {pcd.has_normals()}")
    print(f"  - Has intensity: {'intensity' in pcd.point_attr.keys() if hasattr(pcd, 'point_attr') else 'N/A'}\n")

if __name__ == "__main__":
    check_pcd("pcd/250724/scans2.pcd")
    check_pcd("pcd/250724/scans3.pcd")
