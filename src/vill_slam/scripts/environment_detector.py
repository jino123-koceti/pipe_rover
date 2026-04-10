#!/usr/bin/env python3
"""
Environment Detector Node
Automatically detects if robot is in corridor or pipe environment
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from vill_slam_msgs.msg import LaserLine, SurfaceSection
import numpy as np
from collections import deque


class EnvironmentDetector(Node):
    """Detects environment type from laser line patterns"""

    # Environment modes
    MODE_UNKNOWN = 0
    MODE_CORRIDOR = 1
    MODE_PIPE = 2

    def __init__(self):
        super().__init__('environment_detector')

        # Parameters
        self.declare_parameter('history_size', 30)
        self.declare_parameter('pipe_curvature_threshold', 0.1)
        self.declare_parameter('detection_confidence_threshold', 0.7)
        self.declare_parameter('publish_rate', 1.0)

        self.history_size = self.get_parameter('history_size').value
        self.curvature_threshold = self.get_parameter('pipe_curvature_threshold').value
        self.confidence_threshold = self.get_parameter('detection_confidence_threshold').value

        # History buffers
        self.curvature_history = deque(maxlen=self.history_size)
        self.eccentricity_history = deque(maxlen=self.history_size)
        self.plane_fit_history = deque(maxlen=self.history_size)

        # Current detection
        self.current_mode = self.MODE_UNKNOWN
        self.detection_confidence = 0.0

        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserLine, '/vill_slam/laser_line',
            self.laser_callback, 10)

        self.surface_sub = self.create_subscription(
            SurfaceSection, '/vill_slam/surface_section',
            self.surface_callback, 10)

        # Publishers
        self.mode_pub = self.create_publisher(Int32, '/vill_slam/environment_mode', 10)
        self.mode_str_pub = self.create_publisher(String, '/vill_slam/environment_mode_str', 10)

        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.publish_mode)

        self.get_logger().info('Environment detector initialized')

    def laser_callback(self, msg: LaserLine):
        """Process laser line detection for curvature analysis"""
        if not msg.is_valid or len(msg.image_points) < 10:
            return

        # Extract image points
        points = np.array([[p.x, p.y] for p in msg.image_points])

        # Compute curvature using polynomial fitting
        if len(points) >= 3:
            # Fit 2nd degree polynomial
            try:
                coeffs = np.polyfit(points[:, 0], points[:, 1], 2)
                # Curvature is related to the 2nd derivative
                curvature = abs(2 * coeffs[0])
                self.curvature_history.append(curvature)
            except np.linalg.LinAlgError:
                pass

    def surface_callback(self, msg: SurfaceSection):
        """Process surface section for environment detection"""
        if not msg.is_valid:
            return

        # Store eccentricity (low = circular/pipe, high = planar/corridor)
        self.eccentricity_history.append(msg.eccentricity)

        # Store plane fit quality
        if msg.geometry_type == SurfaceSection.GEOMETRY_PLANE:
            self.plane_fit_history.append(msg.confidence)
        elif msg.geometry_type == SurfaceSection.GEOMETRY_CYLINDER:
            self.plane_fit_history.append(0.0)

    def detect_environment(self):
        """Analyze history to determine environment type"""
        if len(self.curvature_history) < 5:
            self.current_mode = self.MODE_UNKNOWN
            self.detection_confidence = 0.0
            return

        # Compute statistics
        avg_curvature = np.mean(self.curvature_history)
        avg_eccentricity = np.mean(self.eccentricity_history) if self.eccentricity_history else 0.5

        # Decision logic
        # High curvature + low eccentricity = Pipe
        # Low curvature + high eccentricity = Corridor

        pipe_score = 0.0
        corridor_score = 0.0

        # Curvature analysis
        if avg_curvature > self.curvature_threshold:
            pipe_score += 0.4
        else:
            corridor_score += 0.4

        # Eccentricity analysis
        if avg_eccentricity < 0.3:
            pipe_score += 0.3
        elif avg_eccentricity > 0.5:
            corridor_score += 0.3

        # Consistency bonus
        curvature_std = np.std(self.curvature_history)
        if curvature_std < avg_curvature * 0.3:  # Consistent measurements
            if pipe_score > corridor_score:
                pipe_score += 0.2
            else:
                corridor_score += 0.2

        # Plane fit history
        if self.plane_fit_history:
            avg_plane_fit = np.mean(self.plane_fit_history)
            if avg_plane_fit > 0.7:
                corridor_score += 0.1
            elif avg_plane_fit < 0.3:
                pipe_score += 0.1

        # Determine mode
        if pipe_score > corridor_score and pipe_score > self.confidence_threshold:
            self.current_mode = self.MODE_PIPE
            self.detection_confidence = pipe_score
        elif corridor_score > pipe_score and corridor_score > self.confidence_threshold:
            self.current_mode = self.MODE_CORRIDOR
            self.detection_confidence = corridor_score
        else:
            self.current_mode = self.MODE_UNKNOWN
            self.detection_confidence = max(pipe_score, corridor_score)

    def publish_mode(self):
        """Publish current environment detection"""
        self.detect_environment()

        # Publish integer mode
        mode_msg = Int32()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

        # Publish string mode
        mode_str_msg = String()
        if self.current_mode == self.MODE_CORRIDOR:
            mode_str_msg.data = f"CORRIDOR ({self.detection_confidence:.2f})"
        elif self.current_mode == self.MODE_PIPE:
            mode_str_msg.data = f"PIPE ({self.detection_confidence:.2f})"
        else:
            mode_str_msg.data = f"UNKNOWN ({self.detection_confidence:.2f})"
        self.mode_str_pub.publish(mode_str_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
