#!/usr/bin/env python3
"""
VILL-SLAM Monitor UI
PyQt5 based monitoring interface for VILL-SLAM system

Based on Scout Mini implementation:
/home/test/ros2_ws/scout_mini/code/catkin_ws/src/data_manager/scripts/pipe_rover_master.py
"""

import sys
import os

# CRITICAL: Fix Qt plugin conflict between OpenCV and PyQt5
# This MUST be done before ANY imports that might load Qt
# Set to use system Qt plugins explicitly
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms'
# Disable OpenCV's bundled Qt
os.environ['OPENCV_VIDEOIO_PRIORITY_QT'] = '0'
os.environ['QT_DEBUG_PLUGINS'] = '0'

# Standard imports
import subprocess
import signal
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

# Import numpy before cv2
import numpy as np

# Import cv2 with headless mode (no Qt GUI)
# We only need it for image processing, not display
os.environ['OPENCV_VIDEOIO_DEBUG'] = '0'
import cv2

# Now import PyQt5
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QFrame, QGroupBox, QRadioButton,
    QMessageBox
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap, QImage, QColor, QPalette

import subprocess
import signal
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from vill_slam_msgs.msg import VillSlamStatus, SurfaceSection

# cv_bridge for image conversion
from cv_bridge import CvBridge


@dataclass
class SensorStatus:
    """Sensor online/offline status"""
    lidar: bool = False
    imu: bool = False
    line_laser: bool = False
    gps: bool = False
    camera_front: bool = False
    camera_left: bool = False
    camera_right: bool = False


class DataUpdater(QThread):
    """Background thread for ROS data updates"""
    data_updated = pyqtSignal(dict)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.running = True
        self.update_interval = 0.2  # 5 Hz

    def run(self):
        while self.running:
            data = {
                'slam': self.ros_node.slam_data.copy(),
                'sensors': self.ros_node.sensor_status,
                'cameras': self.ros_node.camera_images.copy(),
                'surface': self.ros_node.surface_data.copy()
            }
            self.data_updated.emit(data)
            time.sleep(self.update_interval)

    def stop(self):
        self.running = False


class VillSlamRosNode(Node):
    """ROS2 node for UI data collection"""

    def __init__(self):
        super().__init__('vill_slam_ui_node')

        self.cv_bridge = CvBridge()

        # Data storage
        self.slam_data = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'distance': 0.0
        }
        self.surface_data = {
            'mode': 0,
            'radius': 0.0,
            'confidence': 0.0
        }
        self.sensor_status = SensorStatus()
        self.camera_images: Dict[str, np.ndarray] = {}
        self.last_message_times: Dict[str, float] = {}

        # QoS for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.slam_sub = self.create_subscription(
            Odometry, '/vill_slam/odometry',
            self.slam_callback, 10)

        self.status_sub = self.create_subscription(
            VillSlamStatus, '/vill_slam/status',
            self.status_callback, 10)

        self.surface_sub = self.create_subscription(
            SurfaceSection, '/vill_slam/surface_section',
            self.surface_callback, 10)

        # Camera subscribers
        # ZED-X uses left/image_rect_color, ZED-X One 4K uses gray/rect/image
        for name, topic in [
            ('front', '/zed_front/zed_node/left/image_rect_color'),
            ('left', '/zed_left/zed_node/gray/rect/image'),
            ('right', '/zed_right/zed_node/rgb/rect/image')
        ]:
            self.create_subscription(
                Image, topic,
                lambda msg, n=name: self.camera_callback(msg, n),
                sensor_qos)

    def slam_callback(self, msg: Odometry):
        self.slam_data['x'] = msg.pose.pose.position.x
        self.slam_data['y'] = msg.pose.pose.position.y
        self.slam_data['z'] = msg.pose.pose.position.z
        self.last_message_times['slam'] = time.time()

    def status_callback(self, msg: VillSlamStatus):
        self.slam_data['distance'] = msg.total_distance
        self.surface_data['mode'] = msg.environment_mode
        self.surface_data['radius'] = msg.estimated_pipe_radius

        # Update sensor status from bitmask (use constants directly)
        SENSOR_LIDAR = 1
        SENSOR_IMU = 2
        SENSOR_LINE_LASER = 32
        SENSOR_GPS = 128
        self.sensor_status.lidar = bool(msg.sensors_active & SENSOR_LIDAR)
        self.sensor_status.imu = bool(msg.sensors_active & SENSOR_IMU)
        self.sensor_status.line_laser = bool(msg.sensors_active & SENSOR_LINE_LASER)
        self.sensor_status.gps = bool(msg.sensors_active & SENSOR_GPS)

        self.last_message_times['status'] = time.time()

    def surface_callback(self, msg: SurfaceSection):
        self.surface_data['radius'] = msg.radius
        self.surface_data['confidence'] = msg.confidence
        self.last_message_times['surface'] = time.time()

    def camera_callback(self, msg: Image, camera_name: str):
        try:
            # Handle various image encodings
            if msg.encoding in ['mono8', '8UC1']:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            elif msg.encoding in ['bgra8', 'rgba8']:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'passthrough')
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            else:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Resize for display
            cv_image = cv2.resize(cv_image, (320, 240))
            self.camera_images[camera_name] = cv_image
            self.last_message_times[f'camera_{camera_name}'] = time.time()

            # Update sensor status
            if camera_name == 'front':
                self.sensor_status.camera_front = True
        except Exception as e:
            print(f"Camera {camera_name} error: {e}, encoding: {msg.encoding}")


class LEDIndicator(QLabel):
    """LED status indicator widget"""

    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self.label_text = label
        self.setFixedSize(20, 20)
        self.set_state('offline')

    def set_state(self, state: str):
        """Set LED state: online, offline, warning"""
        colors = {
            'online': '#00ff00',
            'offline': '#ff0000',
            'warning': '#ffff00'
        }
        color = colors.get(state, '#888888')
        self.setStyleSheet(f"""
            background-color: {color};
            border-radius: 10px;
            border: 2px solid #333;
        """)


class VillSlamMonitor(QMainWindow):
    """Main VILL-SLAM monitoring window"""

    def __init__(self):
        super().__init__()

        self.setWindowTitle("VILL-SLAM Monitor")
        self.setGeometry(100, 100, 1200, 800)

        # Initialize ROS2
        rclpy.init()
        self.ros_node = VillSlamRosNode()

        # Start ROS spinner in background
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Data updater
        self.data_updater = DataUpdater(self.ros_node)
        self.data_updater.data_updated.connect(self.update_ui)
        self.data_updater.start()

        # Process management
        self.processes: Dict[str, subprocess.Popen] = {}

        # Build UI
        self.init_ui()

        # LED update timer
        self.led_timer = QTimer()
        self.led_timer.timeout.connect(self.update_leds)
        self.led_timer.start(1000)  # 1 Hz

    def ros_spin(self):
        """Background ROS spinning"""
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    def init_ui(self):
        """Initialize UI components"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)

        # Left panel: Status and Controls
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, 1)

        # Right panel: Cameras and Visualization
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, 2)

    def create_left_panel(self) -> QWidget:
        """Create left status panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Status LEDs
        led_group = QGroupBox("Sensor Status")
        led_layout = QGridLayout()

        self.leds = {}
        sensors = ['LiDAR', 'IMU', 'Line Laser', 'GPS', 'Camera Front']
        for i, name in enumerate(sensors):
            led = LEDIndicator(name)
            label = QLabel(name)
            led_layout.addWidget(led, i, 0)
            led_layout.addWidget(label, i, 1)
            self.leds[name.lower().replace(' ', '_')] = led

        led_group.setLayout(led_layout)
        layout.addWidget(led_group)

        # SLAM State
        slam_group = QGroupBox("SLAM State")
        slam_layout = QGridLayout()

        self.slam_labels = {}
        for i, (key, label) in enumerate([
            ('x', 'X (m):'),
            ('y', 'Y (m):'),
            ('z', 'Z (m):'),
            ('yaw', 'Yaw (°):'),
            ('distance', 'Distance (m):')
        ]):
            slam_layout.addWidget(QLabel(label), i, 0)
            value_label = QLabel('0.00')
            value_label.setStyleSheet("font-weight: bold; font-size: 14px;")
            slam_layout.addWidget(value_label, i, 1)
            self.slam_labels[key] = value_label

        slam_group.setLayout(slam_layout)
        layout.addWidget(slam_group)

        # Environment Mode
        mode_group = QGroupBox("Environment Mode")
        mode_layout = QVBoxLayout()

        self.mode_corridor = QRadioButton("Corridor")
        self.mode_pipe = QRadioButton("Pipe")
        self.mode_auto = QRadioButton("Auto")
        self.mode_auto.setChecked(True)

        mode_layout.addWidget(self.mode_corridor)
        mode_layout.addWidget(self.mode_pipe)
        mode_layout.addWidget(self.mode_auto)

        # Pipe radius display
        self.radius_label = QLabel("Radius: --")
        self.radius_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        mode_layout.addWidget(self.radius_label)

        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        # Control Buttons
        control_group = QGroupBox("Control")
        control_layout = QGridLayout()

        self.btn_start_slam = QPushButton("Start SLAM")
        self.btn_stop_slam = QPushButton("Stop SLAM")
        self.btn_start_record = QPushButton("Start Record")
        self.btn_stop_record = QPushButton("Stop Record")
        self.btn_save_map = QPushButton("Save Map")
        self.btn_reset = QPushButton("Reset")

        self.btn_start_slam.clicked.connect(self.start_slam)
        self.btn_stop_slam.clicked.connect(self.stop_slam)
        self.btn_start_record.clicked.connect(self.start_recording)
        self.btn_stop_record.clicked.connect(self.stop_recording)
        self.btn_save_map.clicked.connect(self.save_map)
        self.btn_reset.clicked.connect(self.reset_slam)

        control_layout.addWidget(self.btn_start_slam, 0, 0)
        control_layout.addWidget(self.btn_stop_slam, 0, 1)
        control_layout.addWidget(self.btn_start_record, 1, 0)
        control_layout.addWidget(self.btn_stop_record, 1, 1)
        control_layout.addWidget(self.btn_save_map, 2, 0)
        control_layout.addWidget(self.btn_reset, 2, 1)

        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        layout.addStretch()
        return panel

    def create_right_panel(self) -> QWidget:
        """Create right camera panel"""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Camera displays
        camera_group = QGroupBox("Camera Feeds")
        camera_layout = QHBoxLayout()

        self.camera_labels = {}
        for name in ['front', 'left', 'right']:
            frame = QFrame()
            frame.setFrameStyle(QFrame.Box)
            frame_layout = QVBoxLayout(frame)

            title = QLabel(name.capitalize())
            title.setAlignment(Qt.AlignCenter)
            frame_layout.addWidget(title)

            image_label = QLabel()
            image_label.setFixedSize(320, 240)
            image_label.setStyleSheet("background-color: #333;")
            image_label.setAlignment(Qt.AlignCenter)
            frame_layout.addWidget(image_label)

            camera_layout.addWidget(frame)
            self.camera_labels[name] = image_label

        camera_group.setLayout(camera_layout)
        layout.addWidget(camera_group)

        # Surface visualization placeholder
        surface_group = QGroupBox("Surface Section")
        surface_layout = QVBoxLayout()

        self.surface_label = QLabel("No data")
        self.surface_label.setAlignment(Qt.AlignCenter)
        self.surface_label.setMinimumHeight(200)
        self.surface_label.setStyleSheet("background-color: #222; color: #fff;")
        surface_layout.addWidget(self.surface_label)

        surface_group.setLayout(surface_layout)
        layout.addWidget(surface_group)

        return panel

    def update_ui(self, data: dict):
        """Update UI with new data"""
        # Update SLAM labels
        slam = data.get('slam', {})
        for key in ['x', 'y', 'z', 'yaw', 'distance']:
            if key in slam and key in self.slam_labels:
                self.slam_labels[key].setText(f"{slam[key]:.2f}")

        # Update surface data
        surface = data.get('surface', {})
        mode = surface.get('mode', 0)
        radius = surface.get('radius', 0)
        confidence = surface.get('confidence', 0)

        mode_str = {0: 'Unknown', 1: 'Corridor', 2: 'Pipe'}.get(mode, 'Unknown')
        self.surface_label.setText(
            f"Mode: {mode_str}\n"
            f"Radius: {radius:.3f} m\n"
            f"Confidence: {confidence:.2f}"
        )

        if radius > 0:
            self.radius_label.setText(f"Radius: {radius:.3f} m")

        # Update camera images
        cameras = data.get('cameras', {})
        for name, image in cameras.items():
            if name in self.camera_labels and image is not None:
                try:
                    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb_image.shape
                    bytes_per_line = ch * w
                    qimg = QImage(rgb_image.data.tobytes(), w, h, bytes_per_line, QImage.Format_RGB888)
                    pixmap = QPixmap.fromImage(qimg)
                    self.camera_labels[name].setPixmap(pixmap)
                except Exception as e:
                    print(f"Display {name} error: {e}")

    def update_leds(self):
        """Update LED indicators based on sensor status from VILL-SLAM"""
        current_time = time.time()
        timeout = 1.5  # seconds

        times = self.ros_node.last_message_times
        sensor_status = self.ros_node.sensor_status

        # Check if we're receiving status messages
        status_alive = (current_time - times.get('status', 0)) < timeout

        # Update LEDs based on sensor_status from VillSlamStatus message
        if status_alive:
            self.leds['lidar'].set_state('online' if sensor_status.lidar else 'offline')
            self.leds['imu'].set_state('online' if sensor_status.imu else 'offline')
            self.leds['line_laser'].set_state('online' if sensor_status.line_laser else 'offline')
            self.leds['gps'].set_state('online' if sensor_status.gps else 'offline')
        else:
            # No status message, all offline
            for led_name in ['lidar', 'imu', 'line_laser', 'gps']:
                if led_name in self.leds:
                    self.leds[led_name].set_state('offline')

        # Camera LED based on image reception
        camera_alive = (current_time - times.get('camera_front', 0)) < timeout
        self.leds['camera_front'].set_state('online' if camera_alive else 'offline')

    # ==================== Control Methods ====================

    def start_slam(self):
        """Start SLAM system - checks if already running first"""
        try:
            # Check if vill_slam_node is already running
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5
            )
            if '/vill_slam_node' in result.stdout:
                self.show_message("SLAM already running (node detected)")
                return

            proc = subprocess.Popen(
                ['ros2', 'launch', 'vill_slam', 'vill_slam.launch.py'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes['slam'] = proc
            self.show_message("SLAM started")
        except Exception as e:
            self.show_message(f"Failed to start SLAM: {e}", error=True)

    def stop_slam(self):
        """Stop SLAM system"""
        if 'slam' in self.processes:
            self.processes['slam'].terminate()
            del self.processes['slam']
            self.show_message("SLAM stopped")

    def start_recording(self):
        """Start data recording"""
        # Call service
        self.show_message("Recording started")

    def stop_recording(self):
        """Stop data recording"""
        self.show_message("Recording stopped")

    def save_map(self):
        """Save current map"""
        self.show_message("Map save triggered")

    def reset_slam(self):
        """Reset SLAM state"""
        self.show_message("SLAM reset")

    def show_message(self, msg: str, error: bool = False):
        """Show status message"""
        if error:
            QMessageBox.warning(self, "Error", msg)
        else:
            self.statusBar().showMessage(msg, 3000)

    def closeEvent(self, event):
        """Handle window close"""
        self.data_updater.stop()
        self.data_updater.wait()

        # Terminate processes
        for proc in self.processes.values():
            proc.terminate()

        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)

    # Set dark theme
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)

    window = VillSlamMonitor()
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
