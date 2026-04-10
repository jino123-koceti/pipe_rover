#!/usr/bin/env python3
"""
Distance-triggered Image Snapshot Node
Captures images at regular distance intervals based on SLAM position
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import math
from datetime import datetime


class ImageSnapshotNode(Node):
    """Captures images at distance-based intervals"""

    def __init__(self):
        super().__init__('image_snapshot')

        # Parameters
        self.declare_parameter('distance_interval', 1.0)  # meters
        self.declare_parameter('save_path', '/mnt/SSD/vill_slam/images/')
        self.declare_parameter('image_format', 'jpg')
        self.declare_parameter('jpeg_quality', 95)

        self.distance_interval = self.get_parameter('distance_interval').value
        self.save_path = self.get_parameter('save_path').value
        self.image_format = self.get_parameter('image_format').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # State
        self.bridge = CvBridge()
        self.last_capture_position = None
        self.total_distance = 0.0
        self.last_position = None
        self.snapshot_count = 0
        self.session_folder = None

        # Latest images from cameras
        self.latest_images = {
            'front': None,
            'left': None,
            'right': None
        }

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/vill_slam/odometry',
            self.odom_callback, 10)

        self.front_sub = self.create_subscription(
            Image, '/zed_front/zed_node/rgb/image_rect_color',
            lambda msg: self.image_callback(msg, 'front'), 10)

        self.left_sub = self.create_subscription(
            Image, '/zed_left/zed_node/rgb/image_rect_color',
            lambda msg: self.image_callback(msg, 'left'), 10)

        self.right_sub = self.create_subscription(
            Image, '/zed_right/zed_node/rgb/image_rect_color',
            lambda msg: self.image_callback(msg, 'right'), 10)

        # Create session folder
        self._create_session_folder()

        self.get_logger().info(
            f'Image snapshot node started (interval: {self.distance_interval}m)')

    def _create_session_folder(self):
        """Create timestamped session folder"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_folder = os.path.join(self.save_path, f'session_{timestamp}')

        try:
            os.makedirs(self.session_folder, exist_ok=True)
            os.makedirs(os.path.join(self.session_folder, 'front'), exist_ok=True)
            os.makedirs(os.path.join(self.session_folder, 'left'), exist_ok=True)
            os.makedirs(os.path.join(self.session_folder, 'right'), exist_ok=True)
            self.get_logger().info(f'Session folder: {self.session_folder}')
        except Exception as e:
            self.get_logger().error(f'Failed to create folder: {e}')

    def image_callback(self, msg: Image, camera_name: str):
        """Store latest image from camera"""
        try:
            self.latest_images[camera_name] = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed ({camera_name}): {e}')

    def odom_callback(self, msg: Odometry):
        """Check distance and capture images"""
        current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        # Update total distance
        if self.last_position is not None:
            dx = current_pos[0] - self.last_position[0]
            dy = current_pos[1] - self.last_position[1]
            dz = current_pos[2] - self.last_position[2]
            self.total_distance += math.sqrt(dx*dx + dy*dy + dz*dz)

        self.last_position = current_pos

        # Check if we need to capture
        if self.last_capture_position is None:
            self._capture_images(current_pos)
        else:
            dx = current_pos[0] - self.last_capture_position[0]
            dy = current_pos[1] - self.last_capture_position[1]
            dz = current_pos[2] - self.last_capture_position[2]
            distance_since_capture = math.sqrt(dx*dx + dy*dy + dz*dz)

            if distance_since_capture >= self.distance_interval:
                self._capture_images(current_pos)

    def _capture_images(self, position):
        """Capture and save images from all cameras"""
        if self.session_folder is None:
            return

        self.last_capture_position = position
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]

        for camera_name, image in self.latest_images.items():
            if image is not None:
                filename = f'{camera_name}_{self.snapshot_count:06d}_{timestamp}.{self.image_format}'
                filepath = os.path.join(self.session_folder, camera_name, filename)

                try:
                    if self.image_format == 'jpg':
                        cv2.imwrite(filepath, image,
                                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                    else:
                        cv2.imwrite(filepath, image)
                except Exception as e:
                    self.get_logger().error(f'Failed to save {camera_name}: {e}')

        self.snapshot_count += 1
        self.get_logger().info(
            f'Snapshot #{self.snapshot_count} at distance {self.total_distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSnapshotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
