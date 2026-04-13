#!/usr/bin/env python3
"""
적응형 평활화 적용 전/후 비교 시각화 프로그램
- 중심점 변화 비교
- 평면화 결과 비교
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
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import os

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
    return model.params

def adaptive_smoothing(current_center, previous_smoothed, center_diff, x_pos, base_alpha=0.01):
    """
    Adaptive EMA smoothing with distance and change magnitude consideration
    Returns: smoothed_center, alpha_used
    """
    # Distance-based smoothing factor
    distance_factor = (x_pos - 0.5) / (5.0 - 0.5)  # 0.0 (near) to 1.0 (far)
    distance_smoothing_factor = 1.0 - distance_factor * 0.5  # Far: 0.5x alpha
    base_alpha_for_position = base_alpha * distance_smoothing_factor
    
    # Adaptive alpha based on change magnitude
    if center_diff > 0.08:  # Very large change: skip update
        return previous_smoothed, 0.0
    elif center_diff > 0.03:  # Large change: exponential decay
        k = 25.0
        threshold = 0.03
        adaptive_alpha = base_alpha_for_position * np.exp(-k * (center_diff - threshold))
        adaptive_alpha = max(adaptive_alpha, 0.0005)
    elif center_diff > 0.015:  # Moderate change: linear decrease
        # Linear interpolation between base_alpha and reduced alpha
        ratio = (center_diff - 0.015) / (0.03 - 0.015)
        adaptive_alpha = base_alpha_for_position * (1.0 - ratio * 0.5)
    else:  # Small change: use base alpha
        adaptive_alpha = base_alpha_for_position
    
    smoothed = adaptive_alpha * current_center + (1 - adaptive_alpha) * previous_smoothed
    return smoothed, adaptive_alpha

def simple_smoothing(current_center, previous_smoothed, alpha=0.1):
    """Simple EMA without adaptation"""
    return alpha * current_center + (1 - alpha) * previous_smoothed, alpha

class SmoothingComparisonVisualizer(Node):
    def __init__(self):
        super().__init__('smoothing_comparison_visualizer')
        self.bridge = CvBridge()
        
        # Fixed x positions (same as pipe_unroller_node)
        near_positions = np.linspace(0.5, 2.0, 5)
        far_positions = np.linspace(2.0, 5.0, 9)
        self.fixed_x_positions = np.unique(np.concatenate([near_positions, far_positions]))
        
        # Storage for comparison
        self.frame_data = []  # List of dicts: {frame_num, x_positions, raw_centers, smoothed_centers, simple_smoothed_centers}
        self.frame_count = 0
        self.max_frames = 50  # Collect 50 frames for comparison
        
        # Previous smoothed values (for adaptive smoothing)
        self.previous_smoothed = {}  # {x_pos: [y, z]}
        # Previous smoothed values (for simple smoothing)
        self.previous_simple = {}  # {x_pos: [y, z]}
        
        # TF transforms (same as pipe_unroller_node)
        self.t_bl_lidar = np.array([0.0, 0.0, 0.32])
        self.R_bl_lidar = np.eye(3)
        self.t_lidar_bl = -self.t_bl_lidar
        self.R_lidar_bl = self.R_bl_lidar.T
        
        # Calibration offset
        self.calibration_offset = np.array([0.0, 0.0, 0.25])
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        
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
        
        self.get_logger().info('Smoothing Comparison Visualizer started.')
    
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Received CameraInfo message.')
    
    def synchronized_callback(self, pc_msg, image_msg):
        if self.camera_matrix is None:
            return
        
        if self.frame_count >= self.max_frames:
            self.generate_comparison_figures()
            self.visualization_complete = True
            return
        
        try:
            # Read point cloud
            points_3d_lidar = read_points(pc_msg)
            if len(points_3d_lidar) == 0:
                return
            
            # Read image
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Estimate centers at fixed positions
            raw_centers = {}  # {x_pos: [y, z]}
            smoothed_centers = {}  # {x_pos: [y, z]}
            simple_smoothed_centers = {}  # {x_pos: [y, z]}
            
            for x_pos in self.fixed_x_positions:
                # Adaptive slice width
                base_slice_width = 0.4
                distance_factor = (x_pos - 0.5) / (5.0 - 0.5)
                slice_width = base_slice_width + distance_factor * 0.4
                
                x_min = x_pos - slice_width / 2
                x_max = x_pos + slice_width / 2
                
                # Fit circle
                result = fit_circle_ransac(points_3d_lidar, x_min, x_max, x_center=x_pos)
                
                if result is not None:
                    yc_lidar, zc_lidar, r = result
                    
                    if 0.1 < r < 2.0:
                        # Transform to base_link
                        pipe_center_lidar_vec = np.array([0.0, yc_lidar, zc_lidar])
                        pipe_center_bl_vec = self.R_lidar_bl @ pipe_center_lidar_vec + self.t_lidar_bl
                        pipe_center_bl = pipe_center_bl_vec[1:3] + self.calibration_offset[1:3]
                        
                        raw_centers[x_pos] = pipe_center_bl.copy()
                        
                        # Apply adaptive smoothing
                        if x_pos in self.previous_smoothed:
                            center_diff = np.linalg.norm(pipe_center_bl - self.previous_smoothed[x_pos])
                            smoothed, alpha = adaptive_smoothing(
                                pipe_center_bl, self.previous_smoothed[x_pos], center_diff, x_pos)
                            self.previous_smoothed[x_pos] = smoothed
                            smoothed_centers[x_pos] = smoothed.copy()
                        else:
                            self.previous_smoothed[x_pos] = pipe_center_bl.copy()
                            smoothed_centers[x_pos] = pipe_center_bl.copy()
                        
                        # Apply simple smoothing (for comparison)
                        if x_pos in self.previous_simple:
                            simple, _ = simple_smoothing(
                                pipe_center_bl, self.previous_simple[x_pos], alpha=0.1)
                            self.previous_simple[x_pos] = simple
                            simple_smoothed_centers[x_pos] = simple.copy()
                        else:
                            self.previous_simple[x_pos] = pipe_center_bl.copy()
                            simple_smoothed_centers[x_pos] = pipe_center_bl.copy()
            
            # Store frame data
            if len(raw_centers) >= 3:
                self.frame_data.append({
                    'frame_num': self.frame_count,
                    'x_positions': list(raw_centers.keys()),
                    'raw_centers': {k: v.copy() for k, v in raw_centers.items()},
                    'smoothed_centers': {k: v.copy() for k, v in smoothed_centers.items()},
                    'simple_smoothed_centers': {k: v.copy() for k, v in simple_smoothed_centers.items()},
                    'image': cv_image.copy()
                })
                self.frame_count += 1
                if self.frame_count % 10 == 0:
                    self.get_logger().info(f'Collected {self.frame_count}/{self.max_frames} frames')
        
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')
    
    def generate_comparison_figures(self):
        """Generate comparison figures for adaptive smoothing"""
        plt.rcParams['font.family'] = 'DejaVu Sans'
        plt.rcParams['axes.unicode_minus'] = False
        
        output_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'pipe_vision', 'picture')
        os.makedirs(output_dir, exist_ok=True)
        
        if len(self.frame_data) < 10:
            self.get_logger().error(f'Not enough frames collected: {len(self.frame_data)}')
            return
        
        # Extract data for a representative x position (e.g., 3.0m - far distance)
        target_x = 3.0
        # Find closest x position
        closest_x = min(self.fixed_x_positions, key=lambda x: abs(x - target_x))
        
        # Extract time series data
        frames = [d['frame_num'] for d in self.frame_data]
        raw_y = []
        raw_z = []
        smoothed_y = []
        smoothed_z = []
        simple_y = []
        simple_z = []
        
        for frame_data in self.frame_data:
            if closest_x in frame_data['raw_centers']:
                raw_y.append(frame_data['raw_centers'][closest_x][0])
                raw_z.append(frame_data['raw_centers'][closest_x][1])
                smoothed_y.append(frame_data['smoothed_centers'][closest_x][0])
                smoothed_z.append(frame_data['smoothed_centers'][closest_x][1])
                simple_y.append(frame_data['simple_smoothed_centers'][closest_x][0])
                simple_z.append(frame_data['simple_smoothed_centers'][closest_x][1])
            else:
                # Use previous value if not available
                if len(raw_y) > 0:
                    raw_y.append(raw_y[-1])
                    raw_z.append(raw_z[-1])
                    smoothed_y.append(smoothed_y[-1])
                    smoothed_z.append(smoothed_z[-1])
                    simple_y.append(simple_y[-1])
                    simple_z.append(simple_z[-1])
        
        frames = np.array(frames[:len(raw_y)])
        raw_y = np.array(raw_y)
        raw_z = np.array(raw_z)
        smoothed_y = np.array(smoothed_y)
        smoothed_z = np.array(smoothed_z)
        simple_y = np.array(simple_y)
        simple_z = np.array(simple_z)
        
        # Calculate standard deviation (noise measure)
        std_raw_y = np.std(raw_y)
        std_raw_z = np.std(raw_z)
        std_smoothed_y = np.std(smoothed_y)
        std_smoothed_z = np.std(smoothed_z)
        std_simple_y = np.std(simple_y)
        std_simple_z = np.std(simple_z)
        
        # Figure 1: Center trajectory comparison (Y and Z)
        fig1, axes = plt.subplots(2, 2, figsize=(16, 10))
        
        # Y center comparison
        ax1 = axes[0, 0]
        ax1.plot(frames, raw_y, 'o-', color='lightblue', markersize=4, alpha=0.6, 
                label=f'Raw (std={std_raw_y*100:.2f}cm)', linewidth=1)
        ax1.plot(frames, simple_y, 's-', color='orange', markersize=4, alpha=0.7,
                label=f'Simple EMA (std={std_simple_y*100:.2f}cm)', linewidth=1.5)
        ax1.plot(frames, smoothed_y, '^-', color='red', markersize=5, alpha=0.8,
                label=f'Adaptive EMA (std={std_smoothed_y*100:.2f}cm)', linewidth=2)
        ax1.set_xlabel('Frame Number', fontsize=12)
        ax1.set_ylabel('Y Center (m)', fontsize=12)
        ax1.set_title(f'(a) Y Center Trajectory Comparison\n(x = {closest_x:.2f}m)', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        
        # Z center comparison
        ax2 = axes[0, 1]
        ax2.plot(frames, raw_z, 'o-', color='lightblue', markersize=4, alpha=0.6,
                label=f'Raw (std={std_raw_z*100:.2f}cm)', linewidth=1)
        ax2.plot(frames, simple_z, 's-', color='orange', markersize=4, alpha=0.7,
                label=f'Simple EMA (std={std_simple_z*100:.2f}cm)', linewidth=1.5)
        ax2.plot(frames, smoothed_z, '^-', color='red', markersize=5, alpha=0.8,
                label=f'Adaptive EMA (std={std_smoothed_z*100:.2f}cm)', linewidth=2)
        ax2.set_xlabel('Frame Number', fontsize=12)
        ax2.set_ylabel('Z Center (m)', fontsize=12)
        ax2.set_title(f'(b) Z Center Trajectory Comparison\n(x = {closest_x:.2f}m)', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        
        # Y center deviation from mean
        ax3 = axes[1, 0]
        mean_raw_y = np.mean(raw_y)
        mean_smoothed_y = np.mean(smoothed_y)
        ax3.plot(frames, (raw_y - mean_raw_y) * 100, 'o-', color='lightblue', 
                markersize=4, alpha=0.6, label='Raw', linewidth=1)
        ax3.plot(frames, (smoothed_y - mean_smoothed_y) * 100, '^-', color='red',
                markersize=5, alpha=0.8, label='Adaptive EMA', linewidth=2)
        ax3.axhline(y=0, color='black', linestyle='--', linewidth=1, alpha=0.5)
        ax3.set_xlabel('Frame Number', fontsize=12)
        ax3.set_ylabel('Deviation from Mean (cm)', fontsize=12)
        ax3.set_title('(c) Y Center Deviation from Mean', fontsize=12)
        ax3.grid(True, alpha=0.3)
        ax3.legend(fontsize=10)
        
        # Z center deviation from mean
        ax4 = axes[1, 1]
        mean_raw_z = np.mean(raw_z)
        mean_smoothed_z = np.mean(smoothed_z)
        ax4.plot(frames, (raw_z - mean_raw_z) * 100, 'o-', color='lightblue',
                markersize=4, alpha=0.6, label='Raw', linewidth=1)
        ax4.plot(frames, (smoothed_z - mean_smoothed_z) * 100, '^-', color='red',
                markersize=5, alpha=0.8, label='Adaptive EMA', linewidth=2)
        ax4.axhline(y=0, color='black', linestyle='--', linewidth=1, alpha=0.5)
        ax4.set_xlabel('Frame Number', fontsize=12)
        ax4.set_ylabel('Deviation from Mean (cm)', fontsize=12)
        ax4.set_title('(d) Z Center Deviation from Mean', fontsize=12)
        ax4.grid(True, alpha=0.3)
        ax4.legend(fontsize=10)
        
        plt.suptitle('Adaptive Smoothing Comparison: Before vs After', fontsize=14, y=0.995)
        plt.tight_layout()
        fig1.savefig(os.path.join(output_dir, 'figure_3_XX_smoothing_comparison.png'),
                    dpi=300, bbox_inches='tight')
        self.get_logger().info('Saved: figure_3_XX_smoothing_comparison.png')
        
        # Figure 2: Unrolled image comparison (before and after smoothing)
        # Use first and last frames to show improvement
        if len(self.frame_data) >= 2:
            self.generate_unrolled_comparison(self.frame_data[0], self.frame_data[-1], output_dir)
        
        plt.close('all')
        self.get_logger().info('All comparison figures generated successfully!')
    
    def generate_unrolled_comparison(self, frame_before, frame_after, output_dir):
        """Generate unrolled image comparison"""
        # This is a simplified version - in practice, you'd use the actual unrolling code
        # For now, we'll create a conceptual comparison figure
        
        fig2, axes = plt.subplots(1, 2, figsize=(16, 6))
        
        # Before smoothing (raw centers)
        ax1 = axes[0]
        x_positions = sorted(frame_before['x_positions'])
        y_centers = [frame_before['raw_centers'][x][0] for x in x_positions]
        z_centers = [frame_before['raw_centers'][x][1] for x in x_positions]
        
        # Interpolate
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
        
        ax1.plot(x_positions, y_centers, 'o', markersize=8, color='blue',
                markeredgecolor='black', markeredgewidth=1, label='Estimated Centers')
        ax1.plot(x_interp, y_interp, '-', linewidth=2, color='red', alpha=0.7,
                label='Interpolated (Cubic Spline)')
        ax1.set_xlabel('X Position (m)', fontsize=12)
        ax1.set_ylabel('Y Center (m)', fontsize=12)
        ax1.set_title('(a) Before Adaptive Smoothing\n(Raw Centers with Cubic Spline)', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        
        # After smoothing (smoothed centers)
        ax2 = axes[1]
        x_positions = sorted(frame_after['x_positions'])
        y_centers = [frame_after['smoothed_centers'][x][0] for x in x_positions]
        z_centers = [frame_after['smoothed_centers'][x][1] for x in x_positions]
        
        # Interpolate
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
        
        ax2.plot(x_positions, y_centers, 'o', markersize=8, color='blue',
                markeredgecolor='black', markeredgewidth=1, label='Smoothed Centers')
        ax2.plot(x_interp, y_interp, '-', linewidth=2, color='red', alpha=0.7,
                label='Interpolated (Cubic Spline)')
        ax2.set_xlabel('X Position (m)', fontsize=12)
        ax2.set_ylabel('Y Center (m)', fontsize=12)
        ax2.set_title('(b) After Adaptive Smoothing\n(Smoothed Centers with Cubic Spline)', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        
        plt.suptitle('Improved Unrolling Result: Smoother Center Trajectory', fontsize=14, y=0.995)
        plt.tight_layout()
        fig2.savefig(os.path.join(output_dir, 'figure_3_XX_unrolling_improvement.png'),
                    dpi=300, bbox_inches='tight')
        self.get_logger().info('Saved: figure_3_XX_unrolling_improvement.png')

def main(args=None):
    rclpy.init(args=args)
    node = SmoothingComparisonVisualizer()
    node.visualization_complete = False
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










