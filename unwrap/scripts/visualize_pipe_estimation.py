#!/usr/bin/env python3
"""
파이프 지오메트리 추정 과정 시각화 프로그램
다중 슬라이스 샘플링 및 RANSAC 기반 원 추정 과정을 그림으로 생성
"""

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import struct
from skimage.measure import CircleModel, ransac
import cv2
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend (no GUI required)
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import matplotlib.patches as mpatches
from scipy.interpolate import interp1d
# Note: Axes3D is not needed for modern matplotlib, just use projection='3d'

# Helper function to parse PointCloud2 data (same as pipe_unroller_node)
def read_points(msg):
    points = []
    point_step = msg.point_step
    row_step = msg.row_step
    data = msg.data
    x_offset, y_offset, z_offset = -1, -1, -1
    for field in msg.fields:
        if field.name == 'x': x_offset = field.offset
        elif field.name == 'y': y_offset = field.offset
        elif field.name == 'z': z_offset = field.offset
    if any(offset == -1 for offset in [x_offset, y_offset, z_offset]):
        raise ValueError("Could not find x, y, z fields in PointCloud2 message")
    for i in range(msg.height):
        for j in range(msg.width):
            base_idx = i * row_step + j * point_step
            x = struct.unpack_from('f', data, base_idx + x_offset)[0]
            y = struct.unpack_from('f', data, base_idx + y_offset)[0]
            z = struct.unpack_from('f', data, base_idx + z_offset)[0]
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z])
    return np.array(points)

def fit_circle_ransac(points_3d, x_min, x_max, x_center=None):
    """Fit a circle to a slice of points between x_min and x_max"""
    pipe_slice = points_3d[(points_3d[:, 0] > x_min) & (points_3d[:, 0] < x_max)]
    if pipe_slice.shape[0] < 10:
        return None 
    points_2d = pipe_slice[:, 1:3]
    
    # Adaptive RANSAC parameters based on distance
    if x_center is not None and x_center > 2.5:
        residual_threshold = 0.18
        max_trials = 400
        min_inlier_ratio = 0.25
    else:
        residual_threshold = 0.15
        max_trials = 300
        min_inlier_ratio = 0.3
    
    model, inliers = ransac(points_2d, CircleModel, min_samples=5, 
                           residual_threshold=residual_threshold, max_trials=max_trials)
    if model is None: 
        return None
    if len(inliers) < len(points_2d) * min_inlier_ratio:
        return None
    return model.params, inliers, points_2d

class PipeEstimationVisualizer(Node):
    def __init__(self):
        super().__init__('pipe_estimation_visualizer')
        self.get_logger().info('Pipe Estimation Visualizer started.')
        self.bridge = CvBridge()
        self.camera_info = None
        
        # Fixed x positions (same as pipe_unroller_node)
        near_positions = np.linspace(0.5, 2.0, 5)
        far_positions = np.linspace(2.0, 5.0, 9)
        self.fixed_x_positions = np.unique(np.concatenate([near_positions, far_positions]))
        
        # TF transforms (same as pipe_unroller_node)
        self.t_bl_lidar = np.array([0.0, 0.0, 0.32])
        self.R_bl_lidar = np.eye(3)
        
        # Storage for visualization
        self.points_3d_lidar = None
        self.estimation_results = {}
        self.frame_count = 0
        self.max_frames = 1  # Process only one frame
        self.visualization_complete = False
        
        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/zed_x_one/camera3/camera_info', 
            self.camera_info_callback, qos_profile=qos_profile_sensor_data)
        self.pc_sub = message_filters.Subscriber(
            self, PointCloud2, '/ouster/points', qos_profile=qos_profile_sensor_data)
        self.image_sub = message_filters.Subscriber(
            self, Image, '/zed_x_one/camera3/image_raw', qos_profile=qos_profile_sensor_data)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.synchronized_callback)
        
    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Received CameraInfo message.')
            self.destroy_subscription(self.camera_info_sub)
    
    def synchronized_callback(self, pc_msg, image_msg):
        if self.camera_info is None:
            return
        
        if self.frame_count >= self.max_frames:
            return
        
        self.frame_count += 1
        self.get_logger().info(f'Processing frame {self.frame_count} for visualization...')
        
        try:
            # Read point cloud
            points_3d_lidar = read_points(pc_msg)
            self.points_3d_lidar = points_3d_lidar
            
            # Transform to base_link
            R_lidar_bl = self.R_bl_lidar.T
            t_lidar_bl = -R_lidar_bl @ self.t_bl_lidar
            
            # Estimate centers at each position
            for x_pos in self.fixed_x_positions:
                # Adaptive slice width
                base_slice_width = 0.4
                distance_factor = (x_pos - 0.5) / (5.0 - 0.5)
                slice_width = base_slice_width + distance_factor * 0.4
                x_min = x_pos - slice_width / 2
                x_max = x_pos + slice_width / 2
                
                # Get slice points
                slice_points = points_3d_lidar[
                    (points_3d_lidar[:, 0] > x_min) & (points_3d_lidar[:, 0] < x_max)]
                
                # Fit circle
                result = fit_circle_ransac(points_3d_lidar, x_min, x_max, x_center=x_pos)
                
                if result is not None:
                    circle_params, inliers, points_2d = result
                    yc_lidar, zc_lidar, r = circle_params
                    
                    if 0.1 < r < 2.0:
                        # Transform to base_link
                        pipe_center_lidar_vec = np.array([0.0, yc_lidar, zc_lidar])
                        pipe_center_bl_vec = R_lidar_bl @ pipe_center_lidar_vec + t_lidar_bl
                        
                        self.estimation_results[x_pos] = {
                            'center_lidar': [yc_lidar, zc_lidar],
                            'center_bl': pipe_center_bl_vec[:2],
                            'radius': r,
                            'slice_points': slice_points,
                            'points_2d': points_2d,
                            'inliers': inliers,
                            'x_min': x_min,
                            'x_max': x_max
                        }
            
            # Generate visualization
            if len(self.estimation_results) > 0:
                self.generate_figures()
                self.get_logger().info('Visualization complete. Figures saved.')
                # Set flag to stop spinning
                self.visualization_complete = True
                
        except Exception as e:
            self.get_logger().error(f'Error in visualization: {e}')
    
    def generate_figures(self):
        """Generate visualization figures"""
        # Set Korean font for matplotlib
        plt.rcParams['font.family'] = 'DejaVu Sans'
        plt.rcParams['axes.unicode_minus'] = False
        
        # Create output directory
        import os
        output_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'pipe_vision', 'picture')
        os.makedirs(output_dir, exist_ok=True)
        
        # Figure 1: 2D views with slices (avoiding 3D projection issues)
        fig1 = plt.figure(figsize=(14, 8))
        
        # Plot all points (subsampled for performance)
        if len(self.points_3d_lidar) > 10000:
            indices = np.random.choice(len(self.points_3d_lidar), 10000, replace=False)
            points_plot = self.points_3d_lidar[indices]
        else:
            points_plot = self.points_3d_lidar
        
        colors = plt.cm.viridis(np.linspace(0, 1, len(self.estimation_results)))
        
        # Subplot 1: X-Y projection (top view)
        ax1 = fig1.add_subplot(121)
        ax1.scatter(points_plot[:, 0], points_plot[:, 1], 
                   c='lightblue', s=0.5, alpha=0.3, label='LiDAR Points')
        
        # Plot slice ranges and centers
        for i, (x_pos, result) in enumerate(sorted(self.estimation_results.items())):
            x_min, x_max = result['x_min'], result['x_max']
            # Draw slice range
            ax1.axvspan(x_min, x_max, alpha=0.2, color=colors[i])
            # Draw center
            ax1.plot(x_pos, result['center_lidar'][0], 'o', 
                    color=colors[i], markersize=8, markeredgecolor='black', markeredgewidth=1,
                    label=f'x={x_pos:.2f}m' if i % 3 == 0 else '')
        
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_title('(a) Multi-slice Sampling Positions\n(X-Y Plane Projection)', fontsize=11, pad=10)
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right', fontsize=8, ncol=2)
        ax1.set_aspect('equal', adjustable='box')
        
        # Subplot 2: X-Z projection (side view)
        ax2 = fig1.add_subplot(122)
        ax2.scatter(points_plot[:, 0], points_plot[:, 2], 
                   c='lightblue', s=0.5, alpha=0.3, label='LiDAR Points')
        
        # Plot slice positions and centers
        for i, (x_pos, result) in enumerate(sorted(self.estimation_results.items())):
            x_min, x_max = result['x_min'], result['x_max']
            # Draw slice range
            ax2.axvspan(x_min, x_max, alpha=0.2, color=colors[i])
            # Draw center
            ax2.plot(x_pos, result['center_lidar'][1], 'o', 
                    color=colors[i], markersize=8, markeredgecolor='black', markeredgewidth=1)
            ax2.text(x_pos, result['center_lidar'][1] + 0.05, f'{x_pos:.1f}m', 
                    fontsize=8, ha='center')
        
        ax2.set_xlabel('X (m)', fontsize=12)
        ax2.set_ylabel('Z (m)', fontsize=12)
        ax2.set_title('(b) Slice Positions and Centers (X-Z Plane Projection)', fontsize=11, pad=10)
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=9)
        ax2.set_aspect('equal', adjustable='box')
        
        plt.tight_layout()
        fig1.savefig(os.path.join(output_dir, 'figure_multislice_sampling.png'), 
                    dpi=300, bbox_inches='tight')
        self.get_logger().info('Saved: figure_multislice_sampling.png')
        
        # Figure 2: RANSAC circle fitting examples
        fig2, axes = plt.subplots(2, 3, figsize=(16, 10))
        axes = axes.flatten()
        
        # Select 6 representative slices to show
        sorted_results = sorted(self.estimation_results.items())
        selected_indices = np.linspace(0, len(sorted_results)-1, 6, dtype=int)
        
        for idx, plot_idx in enumerate(selected_indices):
            if plot_idx >= len(sorted_results):
                break
                
            x_pos, result = sorted_results[plot_idx]
            ax = axes[idx]
            
            points_2d = result['points_2d']
            inliers = result['inliers']
            yc, zc, r = result['center_lidar'][0], result['center_lidar'][1], result['radius']
            
            # Plot all points
            ax.scatter(points_2d[:, 0], points_2d[:, 1], 
                      c='lightblue', s=20, alpha=0.5, label='All Points')
            
            # Plot inliers
            if len(inliers) > 0:
                ax.scatter(points_2d[inliers, 0], points_2d[inliers, 1], 
                          c='green', s=30, alpha=0.7, label='Inliers')
            
            # Plot fitted circle
            circle = Circle((yc, zc), r, fill=False, color='red', linewidth=2, label='Fitted Circle')
            ax.add_patch(circle)
            
            # Plot center
            ax.plot(yc, zc, 'ro', markersize=10, markeredgecolor='black', 
                   markeredgewidth=1, label='Center')
            
            ax.set_xlabel('Y (m)', fontsize=10)
            ax.set_ylabel('Z (m)', fontsize=10)
            ax.set_title(f'x = {x_pos:.2f}m\nr = {r:.3f}m, Points = {len(points_2d)}', 
                        fontsize=10, pad=5)
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal', adjustable='box')
            if idx == 0:
                ax.legend(fontsize=8, loc='upper right')
        
        plt.suptitle('RANSAC-based Pipe Cross-section Circle Estimation (6 Representative Slices)', 
                     fontsize=14, y=0.995)
        plt.tight_layout()
        fig2.savefig(os.path.join(output_dir, 'figure_ransac_circle_fitting.png'), 
                    dpi=300, bbox_inches='tight')
        self.get_logger().info('Saved: figure_ransac_circle_fitting.png')
        
        # Figure 3: Center interpolation
        fig3, axes = plt.subplots(1, 2, figsize=(14, 6))
        
        # Extract centers
        x_positions = []
        y_centers = []
        z_centers = []
        for x_pos in sorted(self.estimation_results.keys()):
            x_positions.append(x_pos)
            center = self.estimation_results[x_pos]['center_bl']
            y_centers.append(center[0])
            z_centers.append(center[1])
        
        x_positions = np.array(x_positions)
        y_centers = np.array(y_centers)
        z_centers = np.array(z_centers)
        
        # Interpolation
        if len(x_positions) >= 4:
            interp_y = interp1d(x_positions, y_centers, kind='cubic', 
                               fill_value='extrapolate', bounds_error=False)
            interp_z = interp1d(x_positions, z_centers, kind='cubic', 
                               fill_value='extrapolate', bounds_error=False)
        else:
            interp_y = interp1d(x_positions, y_centers, kind='linear', 
                               fill_value='extrapolate', bounds_error=False)
            interp_z = interp1d(x_positions, z_centers, kind='linear', 
                               fill_value='extrapolate', bounds_error=False)
        
        x_interp = np.linspace(0.5, 5.0, 100)
        y_interp = interp_y(x_interp)
        z_interp = interp_z(x_interp)
        
        # Plot Y center
        ax1 = axes[0]
        ax1.plot(x_positions, y_centers, 'o', markersize=10, color='blue', 
                markeredgecolor='black', markeredgewidth=1, label='Estimated Centers')
        ax1.plot(x_interp, y_interp, '-', linewidth=2, color='red', 
                alpha=0.7, label='Interpolated Curve (Cubic Spline)')
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Center (m)', fontsize=12)
        ax1.set_title('(a) Y Center Interpolation', fontsize=12, pad=10)
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        
        # Plot Z center
        ax2 = axes[1]
        ax2.plot(x_positions, z_centers, 'o', markersize=10, color='blue', 
                markeredgecolor='black', markeredgewidth=1, label='Estimated Centers')
        ax2.plot(x_interp, z_interp, '-', linewidth=2, color='red', 
                alpha=0.7, label='Interpolated Curve (Cubic Spline)')
        ax2.set_xlabel('X Position (m)', fontsize=12)
        ax2.set_ylabel('Z Center (m)', fontsize=12)
        ax2.set_title('(b) Z Center Interpolation', fontsize=12, pad=10)
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        
        plt.tight_layout()
        fig3.savefig(os.path.join(output_dir, 'figure_center_interpolation.png'), 
                    dpi=300, bbox_inches='tight')
        self.get_logger().info('Saved: figure_center_interpolation.png')
        
        # Combined figure for report
        fig4 = plt.figure(figsize=(16, 10))
        
        # Subplot 1: X-Y projection (top view)
        ax1 = fig4.add_subplot(2, 3, 1)
        if len(self.points_3d_lidar) > 10000:
            indices = np.random.choice(len(self.points_3d_lidar), 10000, replace=False)
            points_plot = self.points_3d_lidar[indices]
        else:
            points_plot = self.points_3d_lidar
        
        ax1.scatter(points_plot[:, 0], points_plot[:, 1], 
                   c='lightblue', s=0.5, alpha=0.3)
        colors = plt.cm.viridis(np.linspace(0, 1, len(self.estimation_results)))
        for i, (x_pos, result) in enumerate(sorted(self.estimation_results.items())):
            x_min, x_max = result['x_min'], result['x_max']
            ax1.axvspan(x_min, x_max, alpha=0.15, color=colors[i])
            ax1.plot(x_pos, result['center_lidar'][0], 'o', 
                    color=colors[i], markersize=6, markeredgecolor='black', markeredgewidth=1)
        ax1.set_xlabel('X (m)', fontsize=10)
        ax1.set_ylabel('Y (m)', fontsize=10)
        ax1.set_title('(a) Multi-slice Sampling\n(13 Fixed Positions, X-Y Plane)', fontsize=10)
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal', adjustable='box')
        
        # Subplot 2: Top view
        ax2 = fig4.add_subplot(2, 3, 2)
        ax2.scatter(points_plot[:, 0], points_plot[:, 1], 
                   c='lightblue', s=0.5, alpha=0.3)
        for i, (x_pos, result) in enumerate(sorted(self.estimation_results.items())):
            x_min, x_max = result['x_min'], result['x_max']
            ax2.axvspan(x_min, x_max, alpha=0.2, color=colors[i])
            ax2.plot(x_pos, result['center_lidar'][0], 'o', 
                    color=colors[i], markersize=8, markeredgecolor='black', markeredgewidth=1)
        ax2.set_xlabel('X (m)', fontsize=10)
        ax2.set_ylabel('Y (m)', fontsize=10)
        ax2.set_title('(b) Slice Positions and Centers', fontsize=10)
        ax2.grid(True, alpha=0.3)
        
        # Subplot 3-5: RANSAC examples (3개)
        for idx, plot_idx in enumerate([0, len(sorted_results)//2, len(sorted_results)-1]):
            if plot_idx >= len(sorted_results):
                continue
            x_pos, result = sorted_results[plot_idx]
            ax = fig4.add_subplot(2, 3, 3+idx)
            
            points_2d = result['points_2d']
            inliers = result['inliers']
            yc, zc, r = result['center_lidar'][0], result['center_lidar'][1], result['radius']
            
            ax.scatter(points_2d[:, 0], points_2d[:, 1], c='lightblue', s=15, alpha=0.5)
            if len(inliers) > 0:
                ax.scatter(points_2d[inliers, 0], points_2d[inliers, 1], 
                          c='green', s=20, alpha=0.7)
            circle = Circle((yc, zc), r, fill=False, color='red', linewidth=2)
            ax.add_patch(circle)
            ax.plot(yc, zc, 'ro', markersize=8, markeredgecolor='black', markeredgewidth=1)
            ax.set_xlabel('Y (m)', fontsize=9)
            ax.set_ylabel('Z (m)', fontsize=9)
            ax.set_title(f'(c{idx+1}) x={x_pos:.2f}m\nr={r:.3f}m', fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal', adjustable='box')
        
        # Subplot 6: Interpolation
        ax6 = fig4.add_subplot(2, 3, 6)
        ax6.plot(x_positions, y_centers, 'o', markersize=8, color='blue', 
                markeredgecolor='black', markeredgewidth=1, label='Estimated Centers')
        ax6.plot(x_interp, y_interp, '-', linewidth=2, color='red', 
                alpha=0.7, label='Interpolated Curve')
        ax6.set_xlabel('X Position (m)', fontsize=10)
        ax6.set_ylabel('Y Center (m)', fontsize=10)
        ax6.set_title('(d) Center Interpolation (Cubic Spline)', fontsize=10)
        ax6.grid(True, alpha=0.3)
        ax6.legend(fontsize=9)
        
        plt.suptitle('Multi-slice Sampling and RANSAC-based Pipe Cross-section Estimation Process', 
                     fontsize=14, y=0.995)
        plt.tight_layout()
        fig4.savefig(os.path.join(output_dir, 'figure_3_XX_pipe_estimation.png'), 
                    dpi=300, bbox_inches='tight')
        self.get_logger().info('Saved: figure_3_XX_pipe_estimation.png (Report Figure)')
        
        plt.close('all')
        self.get_logger().info('All figures generated successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = PipeEstimationVisualizer()
    try:
        while rclpy.ok() and not node.visualization_complete:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

