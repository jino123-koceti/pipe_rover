import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import struct
from skimage.measure import CircleModel, ransac
from geometry_msgs.msg import PointStamped, TransformStamped
import cv2
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d

# Helper function to parse PointCloud2 data
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
    """Fit a circle to a slice of points between x_min and x_max
    x_center: center x position for adaptive parameter adjustment"""
    pipe_slice = points_3d[(points_3d[:, 0] > x_min) & (points_3d[:, 0] < x_max)]
    if pipe_slice.shape[0] < 10:  # Need more points for stable estimation
        return None 
    points_2d = pipe_slice[:, 1:3]
    
    # Adaptive RANSAC parameters based on distance
    # Farther distances: more lenient parameters (higher threshold, more trials)
    if x_center is not None and x_center > 2.5:  # Far distance
        residual_threshold = 0.18  # More lenient for far distances
        max_trials = 400  # More trials for better success rate
        min_inlier_ratio = 0.25  # Lower threshold for far distances
    else:  # Near distance
        residual_threshold = 0.15
        max_trials = 300
        min_inlier_ratio = 0.3
    
    model, inliers = ransac(points_2d, CircleModel, min_samples=5, 
                           residual_threshold=residual_threshold, max_trials=max_trials)
    if model is None: 
        return None
    # Check if enough inliers
    if len(inliers) < len(points_2d) * min_inlier_ratio:
        return None
    return model.params

def estimate_pipe_centers_multislice(points_3d, num_slices=5, x_range=(0.5, 5.0)):
    """Estimate pipe center at multiple x positions using multiple slices"""
    x_min, x_max = x_range
    slice_width = (x_max - x_min) / num_slices
    centers = []
    x_positions = []
    
    for i in range(num_slices):
        slice_x_min = x_min + i * slice_width
        slice_x_max = slice_x_min + slice_width * 1.5  # Overlap for better estimation
        slice_x_center = (slice_x_min + slice_x_max) / 2
        
        circle_params = fit_circle_ransac(points_3d, slice_x_min, slice_x_max)
        if circle_params is not None:
            yc, zc, r = circle_params
            centers.append([yc, zc, r])
            x_positions.append(slice_x_center)
    
    if len(centers) == 0:
        return None
    
    centers = np.array(centers)
    x_positions = np.array(x_positions)
    
    # Use median radius (more stable than mean)
    r_pipe = np.median(centers[:, 2])
    
    return x_positions, centers[:, :2], r_pipe

class PipeUnrollerNode(Node):
    def __init__(self):
        super().__init__('pipe_unroller_node')
        self.get_logger().info('Pipe Unroller Node has been started.')
        self.bridge = CvBridge()
        self.camera_info = None

        # --- Publishers ---
        self.unrolled_pub = self.create_publisher(Image, '/pipe_vision/unrolled_image', 10)
        self.unrolled_raw_pub = self.create_publisher(Image, '/pipe_vision/unrolled_image_raw', 10)

        # --- Subscribers ---
        self.camera_info_sub = self.create_subscription(CameraInfo, '/zed_x_one/camera3/camera_info', self.camera_info_callback, qos_profile=qos_profile_sensor_data)
        self.pc_sub = message_filters.Subscriber(self, PointCloud2, '/ouster/points', qos_profile=qos_profile_sensor_data)
        self.image_sub = message_filters.Subscriber(self, Image, '/zed_x_one/camera3/image_raw', qos_profile=qos_profile_sensor_data)
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.pc_sub, self.image_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.synchronized_callback)
        self.get_logger().info('Synchronized subscribers created. Waiting for messages...')

        # --- Hardcoded TF Transforms (from User) ---
        # Translation and Rotation from base_link to camera3's mounting point (ROS standard frame)
        self.t_bl_cam = np.array([0.15, 0.0, 0.12])
        self.R_bl_cam = np.eye(3) # No rotation relative to base_link

        # Translation and Rotation from base_link to LiDAR (ROS standard frame)
        self.t_bl_lidar = np.array([0.0, 0.0, 0.32])
        self.R_bl_lidar = np.eye(3)

        # Standard rotation from a ROS camera frame (X-fwd) to an OpenCV optical frame (Z-fwd)
        self.R_cam_to_optical = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])

        # Smoothing factor for the pipe center (lower = more smoothing, more stable)
        self.smoothed_pipe_center = None
        self.smoothing_alpha = 0.01  # Further reduced for maximum stability (was 0.02)
        
        # Fixed x positions for center estimation (in base_link frame)
        # More positions with higher density at far distances for better accuracy
        # Use non-linear spacing: more points at far distances
        near_positions = np.linspace(0.5, 2.0, 5)  # 5 points in near range (0.5-2.0m)
        far_positions = np.linspace(2.0, 5.0, 9)   # 9 points in far range (2.0-5.0m) - higher density
        self.fixed_x_positions = np.unique(np.concatenate([near_positions, far_positions]))  # Total ~13 positions
        
        # Store smoothed centers for each fixed x position
        self.smoothed_centers_dict = {}  # key: x_position, value: [y, z]
        self.failure_count_dict = {}  # key: x_position, value: consecutive failure count
        self.smoothed_radius = None
        
        # Debug logging
        self.debug_logging = True
        self.frame_count = 0
        
        # Log file for detailed debugging
        import os
        from datetime import datetime
        log_dir = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'log', 'pipe_vision')
        os.makedirs(log_dir, exist_ok=True)
        log_filename = f"pipe_unroller_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        self.log_file_path = os.path.join(log_dir, log_filename)
        self.log_file = open(self.log_file_path, 'w')
        self.get_logger().info(f'Debug log file: {self.log_file_path}')
        
        # Manual calibration offset to correct for TF inaccuracies
        self.calibration_offset = np.array([0.0, 0.0, 0.25]) # x, y, z offset in meters

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Received CameraInfo message and stored it.')
            self.destroy_subscription(self.camera_info_sub)

    def synchronized_callback(self, pc_msg, image_msg):
        if self.camera_info is None:
            self.get_logger().warn('Waiting for CameraInfo message...', throttle_duration_sec=5)
            return 

        try:
            # --- Step 1 & 2: Get Geometry and Pose ---
            points_3d_lidar = read_points(pc_msg)
            
            # Estimate pipe centers at fixed x positions
            R_lidar_bl = self.R_bl_lidar.T
            t_lidar_bl = -R_lidar_bl @ self.t_bl_lidar
            
            current_centers = {}
            radii = []
            estimation_stats = {}  # For debugging
            
            # Estimate center at each fixed x position
            for x_pos in self.fixed_x_positions:
                # Use adaptive slice width: wider slice for farther distances (more points, more stable)
                # Closer: 0.4m, Farther: 0.8m (increased from 0.7m)
                base_slice_width = 0.4
                distance_factor = (x_pos - 0.5) / (5.0 - 0.5)  # 0.0 to 1.0
                slice_width = base_slice_width + distance_factor * 0.4  # 0.4m to 0.8m
                x_min = x_pos - slice_width / 2
                x_max = x_pos + slice_width / 2
                
                # Count points in slice
                slice_points = points_3d_lidar[(points_3d_lidar[:, 0] > x_min) & (points_3d_lidar[:, 0] < x_max)]
                num_points = len(slice_points)
                
                circle_params = fit_circle_ransac(points_3d_lidar, x_min, x_max, x_center=x_pos)
                if circle_params is not None:
                    yc_lidar, zc_lidar, r = circle_params
                    
                    # Filter out unreasonable radii (too small or too large)
                    if 0.1 < r < 2.0:  # Reasonable pipe radius range
                        # Transform to base_link frame
                        pipe_center_lidar_vec = np.array([0.0, yc_lidar, zc_lidar])
                        pipe_center_bl_vec = R_lidar_bl @ pipe_center_lidar_vec + t_lidar_bl
                        
                        current_centers[x_pos] = pipe_center_bl_vec[:2]  # [y, z]
                        radii.append(r)
                        estimation_stats[x_pos] = {'success': True, 'num_points': num_points, 'radius': r, 
                                                  'center_lidar': [yc_lidar, zc_lidar], 
                                                  'center_bl': pipe_center_bl_vec[:2].tolist()}
                        # Reset failure count on success
                        if x_pos in self.failure_count_dict:
                            self.failure_count_dict[x_pos] = 0
                    else:
                        estimation_stats[x_pos] = {'success': False, 'reason': f'radius_out_of_range: {r:.3f}', 'num_points': num_points}
                else:
                    estimation_stats[x_pos] = {'success': False, 'reason': 'ransac_failed', 'num_points': num_points}
            
            self.frame_count += 1
            
            # If all centers failed, use previous smoothed values
            if len(current_centers) == 0:
                self.get_logger().warn(f'[Frame {self.frame_count}] Pipe Geometry FAILED: All RANSAC failed. Using previous smoothed values.', throttle_duration_sec=5)
                # Continue with previous smoothed values if available
                if len(self.smoothed_centers_dict) < 3:
                    self.get_logger().error(f'[Frame {self.frame_count}] No previous values available. Skipping frame.')
                    return
                # Use all previous smoothed values
                current_centers = {}  # Empty, will use previous values in smoothing loop
            
            # Use median radius (or previous value if all failed)
            if radii:
                r_pipe = np.median(radii)
                self.smoothed_radius = r_pipe
            else:
                # Use previous radius if available
                r_pipe = self.smoothed_radius if self.smoothed_radius is not None else 0.3
            
            # Smooth centers at each fixed x position independently
            # Use previous frame's value if current estimation failed (more stable)
            smoothed_centers_list = []
            x_positions_list = []
            
            for x_pos in self.fixed_x_positions:
                if x_pos in current_centers:
                    # Current frame successfully estimated center
                    current_center = current_centers[x_pos]
                    
                    # Smooth this position
                    if x_pos not in self.smoothed_centers_dict:
                        self.smoothed_centers_dict[x_pos] = current_center.copy()
                        if self.debug_logging:
                            self.get_logger().info(f'[Frame {self.frame_count}] x={x_pos:.2f}m: Initialized center={current_center}')
                    else:
                        old_center = self.smoothed_centers_dict[x_pos].copy()
                        center_diff = np.linalg.norm(current_center - old_center)
                        
                        # Adaptive smoothing: continuous adjustment based on change magnitude
                        # Use exponential decay: larger changes -> smaller alpha
                        if center_diff > 0.08:  # If change > 8cm, skip update (too unreliable) - lowered from 10cm
                            if self.debug_logging:
                                self.get_logger().warn(f'[Frame {self.frame_count}] x={x_pos:.2f}m: Very large change detected ({center_diff:.4f}m), skipping update to maintain stability')
                                self.log_file.write(f'[Frame {self.frame_count}] x={x_pos:.2f}m: Very large change {center_diff:.4f}m, SKIPPED update. old={old_center}, new={current_center}\n')
                                self.log_file.flush()
                            # Skip update, keep previous value (don't update smoothed_centers_dict)
                            # smoothed_centers_dict[x_pos] remains unchanged
                        else:
                            # Calculate adaptive alpha based on change magnitude AND distance
                            # Farther distances need stronger smoothing (more stable)
                            distance_factor = (x_pos - 0.5) / (5.0 - 0.5)  # 0.0 (near) to 1.0 (far)
                            distance_smoothing_factor = 1.0 - distance_factor * 0.5  # Far: 0.5x alpha, Near: 1.0x alpha
                            base_alpha_for_position = self.smoothing_alpha * distance_smoothing_factor
                            
                            if center_diff > 0.03:  # If change > 3cm, use very strong smoothing (lowered from 5cm)
                                # Exponential decay: alpha decreases exponentially with change magnitude
                                # alpha = base_alpha * exp(-k * (diff - threshold))
                                k = 25.0  # Increased decay rate for faster reduction
                                threshold = 0.03
                                adaptive_alpha = base_alpha_for_position * np.exp(-k * (center_diff - threshold))
                                adaptive_alpha = max(adaptive_alpha, 0.0005)  # Lower minimum alpha
                                if self.debug_logging:
                                    self.get_logger().warn(f'[Frame {self.frame_count}] x={x_pos:.2f}m: Large change detected ({center_diff:.4f}m), using adaptive smoothing (alpha={adaptive_alpha:.4f}, dist_factor={distance_factor:.2f})')
                                    self.log_file.write(f'[Frame {self.frame_count}] x={x_pos:.2f}m: Large change {center_diff:.4f}m, adaptive_alpha={adaptive_alpha:.4f}, dist_factor={distance_factor:.2f}, old={old_center}, new={current_center}\n')
                                    self.log_file.flush()
                            elif center_diff > 0.015:  # If change > 1.5cm, use moderate smoothing (lowered from 2cm)
                                # Linear interpolation between normal and reduced alpha
                                adaptive_alpha = base_alpha_for_position * (1.0 - (center_diff - 0.015) / 0.015 * 0.6)  # More reduction
                            else:
                                adaptive_alpha = base_alpha_for_position
                            
                            # Update smoothed center
                            self.smoothed_centers_dict[x_pos] = (
                                adaptive_alpha * current_center +
                                (1.0 - adaptive_alpha) * self.smoothed_centers_dict[x_pos]
                            )
                        if self.debug_logging and self.frame_count % 10 == 0:
                            self.get_logger().info(f'[Frame {self.frame_count}] x={x_pos:.2f}m: current={current_center}, smoothed={self.smoothed_centers_dict[x_pos]}, diff={center_diff:.4f}m')
                            self.log_file.write(f'[Frame {self.frame_count}] x={x_pos:.2f}m: current={current_center}, smoothed={self.smoothed_centers_dict[x_pos]}, diff={center_diff:.4f}m\n')
                            self.log_file.flush()
                    
                    # Reset failure count
                    if x_pos in self.failure_count_dict:
                        self.failure_count_dict[x_pos] = 0
                else:
                    # Current frame failed - use previous smoothed value if available
                    if x_pos not in self.smoothed_centers_dict:
                        # No previous value, skip this position
                        if self.debug_logging:
                            self.get_logger().warn(f'[Frame {self.frame_count}] x={x_pos:.2f}m: No previous value, skipping. Reason: {estimation_stats.get(x_pos, {}).get("reason", "unknown")}')
                        continue
                    
                    # Increment failure count
                    if x_pos not in self.failure_count_dict:
                        self.failure_count_dict[x_pos] = 0
                    self.failure_count_dict[x_pos] += 1
                    
                    # Keep using previous smoothed value (don't update)
                    if self.debug_logging and self.failure_count_dict[x_pos] % 5 == 0:
                        self.get_logger().warn(f'[Frame {self.frame_count}] x={x_pos:.2f}m: Using previous value (failures={self.failure_count_dict[x_pos]}). Reason: {estimation_stats.get(x_pos, {}).get("reason", "unknown")}, points={estimation_stats.get(x_pos, {}).get("num_points", 0)}')
                
                # Apply calibration offset and add to list
                calibrated_center = self.smoothed_centers_dict[x_pos] + self.calibration_offset[1:3]
                smoothed_centers_list.append(calibrated_center)
                x_positions_list.append(x_pos)
            
            if len(smoothed_centers_list) < 3:
                # Need at least 3 points for stable interpolation
                success_count = len(current_centers)
                self.get_logger().warn(f'[Frame {self.frame_count}] Not enough centers ({success_count} succeeded, {len(smoothed_centers_list)} available) for interpolation.', throttle_duration_sec=2)
                if self.debug_logging:
                    for x_pos, stats in estimation_stats.items():
                        if not stats.get('success', False):
                            self.get_logger().warn(f'  x={x_pos:.2f}m: {stats}')
                return
            
            # Debug log summary every 20 frames
            if self.debug_logging and self.frame_count % 20 == 0:
                success_count = len(current_centers)
                failure_info = []
                for x_pos in self.fixed_x_positions:
                    if x_pos not in current_centers:
                        fail_count = self.failure_count_dict.get(x_pos, 0)
                        failure_info.append(f'x={x_pos:.2f}m(fails={fail_count})')
                summary_msg = f'[Frame {self.frame_count}] Summary: {success_count}/5 succeeded, r={r_pipe:.3f}m. Failures: {", ".join(failure_info) if failure_info else "none"}'
                self.get_logger().info(summary_msg)
                self.log_file.write(summary_msg + '\n')
                self.log_file.flush()
            
            # Convert to arrays for interpolation
            x_positions_bl = np.array(x_positions_list)
            smoothed_centers_calibrated = np.array(smoothed_centers_list)  # Shape: (N, 2)

            # --- Step 3: Image Unrolling ---
            K = np.array(self.camera_info.k).reshape(3, 3)
            D = np.array(self.camera_info.d)
            
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            img_height, img_width = cv_image.shape[:2]
            
            # Standard resolution (for /pipe_vision/unrolled_image)
            unrolled_width = int(2 * np.pi * r_pipe * 400)
            unrolled_height = 400
            
            # High resolution (for /pipe_vision/unrolled_image_raw) - based on original image resolution
            # Use higher resolution to preserve detail: match or exceed original image resolution
            # Height: use original image height or higher
            # Width: maintain aspect ratio based on pipe circumference
            unrolled_height_raw = img_height  # Use original image height
            unrolled_width_raw = int(2 * np.pi * r_pipe * (unrolled_height_raw / 4.5))  # Maintain pixel density
            
            # Function to create unrolled image with given resolution
            def create_unrolled_image(target_height, target_width):
                map_x = np.zeros((target_height, target_width), dtype=np.float32)
                map_y = np.zeros((target_height, target_width), dtype=np.float32)

                # Create the grid of points for the unrolled image
                v_unrolled, u_unrolled = np.mgrid[:target_height, :target_width]

                # Map unrolled pixel coordinates to 3D points on the pipe surface
                theta = (u_unrolled / target_width) * (2 * np.pi)
                x_pipe = 0.5 + (v_unrolled / target_height) * 4.5

                # Interpolate pipe center for each x position
                # Use cubic spline interpolation for smoother results, especially at far distances
                # Interpolate y and z centers separately
                if len(x_positions_bl) >= 4:
                    # Use cubic spline if we have enough points (smoother, better for far distances)
                    interp_y = interp1d(x_positions_bl, smoothed_centers_calibrated[:, 0], 
                                     kind='cubic', fill_value='extrapolate', bounds_error=False)
                    interp_z = interp1d(x_positions_bl, smoothed_centers_calibrated[:, 1], 
                                     kind='cubic', fill_value='extrapolate', bounds_error=False)
                else:
                    # Fall back to linear if not enough points
                    interp_y = interp1d(x_positions_bl, smoothed_centers_calibrated[:, 0], 
                                     kind='linear', fill_value='extrapolate', bounds_error=False)
                    interp_z = interp1d(x_positions_bl, smoothed_centers_calibrated[:, 1], 
                                     kind='linear', fill_value='extrapolate', bounds_error=False)
                
                # Get interpolated centers for each x_pipe value
                yc_interp = interp_y(x_pipe)
                zc_interp = interp_z(x_pipe)

                # Calculate 3D points on the pipe surface in the base_link frame
                y_pipe = yc_interp + r_pipe * np.cos(theta)
                z_pipe = zc_interp + r_pipe * np.sin(theta)
                
                # Assemble 3D points in base_link frame (N, 3)
                P_pipe_bl = np.stack([x_pipe.ravel(), y_pipe.ravel(), z_pipe.ravel()], axis=-1)

                # Transform points from base_link to the ROS camera frame
                P_pipe_cam_ros = (self.R_bl_cam.T @ (P_pipe_bl - self.t_bl_cam).T).T

                # Transform points from ROS camera frame to OpenCV optical frame
                P_pipe_cam_cv = (self.R_cam_to_optical @ P_pipe_cam_ros.T).T

                # Project 3D points onto the 2D image plane
                # We only need rvec and tvec if the points are not already in the camera's coordinate system
                # Since P_pipe_cam_cv is in the camera's optical frame, rvec and tvec are zero.
                image_points, _ = cv2.projectPoints(P_pipe_cam_cv, np.zeros(3), np.zeros(3), K, D)
                
                # Reshape the projected points to match the map dimensions
                map_x = image_points[:, 0, 0].reshape(target_height, target_width)
                map_y = image_points[:, 0, 1].reshape(target_height, target_width)

                # Ensure maps are float32 for cv2.remap
                map_x = map_x.astype(np.float32)
                map_y = map_y.astype(np.float32)

                # Perform the remap operation with higher quality interpolation
                unrolled_image = cv2.remap(cv_image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
                
                return unrolled_image

            # Create standard resolution image
            unrolled_image = create_unrolled_image(unrolled_height, unrolled_width)
            
            # Create high resolution image (raw)
            unrolled_image_raw = create_unrolled_image(unrolled_height_raw, unrolled_width_raw)
            
            # Flip raw image vertically for easier comparison with original pipe image
            unrolled_image_raw = cv2.flip(unrolled_image_raw, 0)  # 0 = flip vertically

            self.get_logger().info(f'cv_image shape: {cv_image.shape}, standard: {unrolled_height}x{unrolled_width}, raw: {unrolled_height_raw}x{unrolled_width_raw}', throttle_duration_sec=1)

            # Publish both images
            self.unrolled_pub.publish(self.bridge.cv2_to_imgmsg(unrolled_image, "bgr8"))
            self.unrolled_raw_pub.publish(self.bridge.cv2_to_imgmsg(unrolled_image_raw, "bgr8"))
            self.get_logger().info('Published unrolled images: standard and raw', throttle_duration_sec=5)

        except Exception as e:
            self.get_logger().error(f'An error occurred in synchronized_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PipeUnrollerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close log file
        if hasattr(node, 'log_file') and node.log_file:
            node.log_file.close()
            node.get_logger().info(f'Log file closed: {node.log_file_path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()