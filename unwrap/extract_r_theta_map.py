import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def extract_radius_by_theta(pcd_path, num_bins=360, r_min=1.4, r_max=1.6):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    x, y = points[:, 0], points[:, 1]
    theta = (np.arctan2(y, x) + 2 * np.pi) % (2 * np.pi)
    r = np.sqrt(x**2 + y**2)

    theta_bins = np.linspace(0, 2 * np.pi, num_bins + 1)
    r_by_theta = []

    for i in range(num_bins):
        mask = (theta >= theta_bins[i]) & (theta < theta_bins[i + 1])
        if np.any(mask):
            r_filtered = r[mask]
            r_filtered = r_filtered[(r_filtered >= r_min) & (r_filtered <= r_max)]

            if len(r_filtered) > 0:
                r_val = np.median(r_filtered)
            else:
                r_val = np.nan  # 비었을 경우 NaN 처리
        else:
            r_val = np.nan

        r_by_theta.append(r_val)

    r_by_theta = np.array(r_by_theta)

    # NaN 보간
    nan_mask = np.isnan(r_by_theta)
    if np.any(nan_mask):
        r_by_theta[nan_mask] = np.interp(np.flatnonzero(nan_mask),
                                         np.flatnonzero(~nan_mask),
                                         r_by_theta[~nan_mask])

    np.save("radius_by_theta_filtered.npy", r_by_theta)

    plt.plot(np.rad2deg(theta_bins[:-1]), r_by_theta)
    plt.xlabel("Theta (deg)")
    plt.ylabel("Radius (m)")
    plt.title("Filtered radius by theta (valid only for 1.4m~1.6m)")
    plt.grid(True)
    plt.savefig("radius_by_theta_filtered.png")
    plt.close()

    print("✅ Saved filtered radius_by_theta_filtered.npy and .png")


if __name__ == "__main__":
    extract_radius_by_theta("pcd/250724/scans2.pcd")
