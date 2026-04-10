#!/usr/bin/env python3
"""
Laser synchronization node for alternating laser on/off with camera frames
Synchronizes line laser with ZED-X camera capture
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Image
from vill_slam_msgs.msg import LaserLine
from cv_bridge import CvBridge
import numpy as np
import cv2
import threading
import time


class LaserSyncNode(Node):
    """Synchronizes laser on/off with camera frames for alternating capture"""

    def __init__(self):
        super().__init__('laser_sync_node')

        # Parameters
        self.declare_parameter('camera_topic', '/zed_front/zed_node/rgb/image_rect_color')
        self.declare_parameter('target_fps', 30.0)
        self.declare_parameter('laser_on_frames', 1)  # Frames with laser on
        self.declare_parameter('laser_off_frames', 1)  # Frames with laser off
        self.declare_parameter('sync_mode', 'alternating')  # alternating, continuous, manual
        self.declare_parameter('warmup_frames', 5)  # Initial frames to skip

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.target_fps = self.get_parameter('target_fps').value
        self.laser_on_frames = self.get_parameter('laser_on_frames').value
        self.laser_off_frames = self.get_parameter('laser_off_frames').value
        self.sync_mode = self.get_parameter('sync_mode').value
        self.warmup_frames = self.get_parameter('warmup_frames').value

        # State
        self.frame_count = 0
        self.laser_frame_counter = 0
        self.is_laser_on = False
        self.is_initialized = False
        self.last_frame_time = None
        self.cv_bridge = CvBridge()

        # Frame buffers
        self.visual_frame = None
        self.laser_frame = None
        self.visual_frame_time = None
        self.laser_frame_time = None
        self.buffer_lock = threading.Lock()

        # QoS for camera subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, self.camera_topic, self._camera_callback, qos)

        # Publishers
        self.laser_cmd_pub = self.create_publisher(Bool, 'line_laser/laser1_cmd', 10)
        self.visual_frame_pub = self.create_publisher(Image, 'vill_slam/visual_frame', 10)
        self.laser_frame_pub = self.create_publisher(Image, 'vill_slam/laser_frame', 10)
        self.sync_status_pub = self.create_publisher(Bool, 'vill_slam/sync_active', 10)

        # Timer for sync control
        if self.sync_mode == 'alternating':
            self.sync_timer = self.create_timer(1.0 / (self.target_fps * 2), self._sync_callback)
        else:
            self.sync_timer = None

        self.get_logger().info(f'Laser sync node initialized')
        self.get_logger().info(f'  Camera topic: {self.camera_topic}')
        self.get_logger().info(f'  Sync mode: {self.sync_mode}')
        self.get_logger().info(f'  Target FPS: {self.target_fps}')

    def _camera_callback(self, msg: Image):
        """Process incoming camera frames"""
        self.frame_count += 1

        # Skip warmup frames
        if self.frame_count <= self.warmup_frames:
            return

        if not self.is_initialized:
            self.is_initialized = True
            self.get_logger().info('Sync initialized, starting capture')

        current_time = self.get_clock().now()

        # Store frame based on current laser state
        with self.buffer_lock:
            if self.is_laser_on:
                self.laser_frame = msg
                self.laser_frame_time = current_time
                self.laser_frame_pub.publish(msg)
            else:
                self.visual_frame = msg
                self.visual_frame_time = current_time
                self.visual_frame_pub.publish(msg)

        self.last_frame_time = current_time

    def _sync_callback(self):
        """Timer callback for alternating laser control"""
        if not self.is_initialized:
            return

        self.laser_frame_counter += 1

        # Determine if laser should be on or off
        cycle_length = self.laser_on_frames + self.laser_off_frames
        position_in_cycle = self.laser_frame_counter % cycle_length

        new_state = position_in_cycle < self.laser_on_frames

        if new_state != self.is_laser_on:
            self.is_laser_on = new_state
            cmd_msg = Bool()
            cmd_msg.data = self.is_laser_on
            self.laser_cmd_pub.publish(cmd_msg)

        # Publish sync status
        status_msg = Bool()
        status_msg.data = True
        self.sync_status_pub.publish(status_msg)

    def get_synchronized_pair(self):
        """Get a synchronized pair of visual and laser frames"""
        with self.buffer_lock:
            if self.visual_frame is None or self.laser_frame is None:
                return None, None

            # Check temporal proximity
            if self.visual_frame_time is None or self.laser_frame_time is None:
                return None, None

            time_diff = abs((self.visual_frame_time - self.laser_frame_time).nanoseconds / 1e9)

            # Frames should be within 100ms of each other
            if time_diff > 0.1:
                return None, None

            return self.visual_frame, self.laser_frame


def main(args=None):
    rclpy.init(args=args)
    node = LaserSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Turn off laser on shutdown
        cmd_msg = Bool()
        cmd_msg.data = False
        node.laser_cmd_pub.publish(cmd_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
