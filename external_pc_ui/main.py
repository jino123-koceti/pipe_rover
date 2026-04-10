#!/usr/bin/env python3
"""VILL-SLAM 외부 모니터링 UI (Zenoh + PyQt5 + pyqtgraph)."""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from zenoh_receiver import ZenohReceiver


def intensity_to_colormap(intensity: np.ndarray) -> np.ndarray:
    i = np.asarray(intensity, dtype=np.float64)
    n = i.size
    if n == 0:
        return np.zeros((0, 4), dtype=np.float32)
    lo, hi = np.percentile(i, [2.0, 98.0])
    if hi <= lo:
        t = np.zeros(n, dtype=np.float32)
    else:
        t = np.clip((i.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)
    r = np.clip(1.5 - 4 * np.abs(t - 0.75), 0, 1)
    g = np.clip(1.5 - 4 * np.abs(t - 0.5), 0, 1)
    b = np.clip(1.5 - 4 * np.abs(t - 0.25), 0, 1)
    return np.stack([r, g, b, np.ones(n, dtype=np.float32)], axis=1).astype(np.float32)


def _rgb_field_to_rgba_float(rgb_raw, n: int) -> tuple[np.ndarray | None, str]:
    """
    로봇 브릿지가 보내는 rgb 바이트를 (n,4) RGBA float로 변환.
    지원: (N,3) uint8, (N,4) uint8 RGBA, (N,) uint32 PCL/ROS packed, (N,) float32 동일 비트패턴.
    """
    if rgb_raw is None or n <= 0:
        return None, ""

    if isinstance(rgb_raw, (list, tuple)):
        flat = np.asarray(rgb_raw, dtype=np.uint8).ravel()
        b = flat.tobytes()
    elif isinstance(rgb_raw, (bytes, bytearray, memoryview)):
        b = bytes(rgb_raw)
    else:
        try:
            b = bytes(memoryview(rgb_raw))
        except TypeError:
            return None, ""

    blen = len(b)
    if blen == 0:
        return None, ""

    out = np.ones((n, 4), dtype=np.float32)

    if blen == n * 3:
        rgb = np.frombuffer(b, dtype=np.uint8).reshape(n, 3)
        out[:, :3] = rgb.astype(np.float32) / 255.0
        return out, "RGB N×3 (uint8)"

    if blen == n * 4:
        u8 = np.frombuffer(b, dtype=np.uint8).reshape(n, 4)
        c_u8 = u8[:, :3].astype(np.float32) / 255.0
        u32 = np.frombuffer(b, dtype=np.uint32)
        r = ((u32 >> 16) & 0xFF).astype(np.float32) / 255.0
        g = ((u32 >> 8) & 0xFF).astype(np.float32) / 255.0
        bch = (u32 & 0xFF).astype(np.float32) / 255.0
        c_u32 = np.stack([r, g, bch], axis=1)

        def _mean_sat(c: np.ndarray) -> float:
            return float(np.mean(np.max(c, axis=1) - np.min(c, axis=1)))

        s8, s32 = _mean_sat(c_u8), _mean_sat(c_u32)
        out_u8 = np.ones((n, 4), dtype=np.float32)
        out_u8[:, :3] = c_u8
        out_u32 = np.ones((n, 4), dtype=np.float32)
        out_u32[:, :3] = c_u32
        # ROS PointField rgb(float)도 동일 4바이트 비트패턴
        if s32 > s8 + 0.015:
            return out_u32, "RGB packed uint32 / float 비트 (ROS·PCL)"
        return out_u8, "RGB N×4 (uint8)"

    return None, f"rgb 길이 불일치 ({blen} bytes, 점 {n}개)"


DEFAULT_ZENOH_PORT = 7447


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VILL-SLAM Monitor")
        self.resize(1400, 900)

        self._port = DEFAULT_ZENOH_PORT
        self._receiver: ZenohReceiver | None = None
        self._record_on = False
        self._last_pose: dict | None = None
        self._last_status: dict | None = None

        pg.setConfigOptions(antialias=True, background="k", foreground="w")

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # 상단: 연결 바
        bar = QHBoxLayout()
        self.title_lbl = QLabel("VILL-SLAM Monitor")
        f = QFont()
        f.setPointSize(12)
        f.setBold(True)
        self.title_lbl.setFont(f)
        bar.addWidget(self.title_lbl)
        bar.addStretch()
        bar.addWidget(QLabel("로봇 IP:"))
        self.ip_edit = QLineEdit()
        self.ip_edit.setPlaceholderText("예: 192.168.0.10")
        self.ip_edit.setFixedWidth(200)
        self.ip_edit.setToolTip("로봇(Jetson)의 IP 주소를 입력한 뒤 연결을 누르세요.")
        bar.addWidget(self.ip_edit)
        bar.addWidget(QLabel("포트:"))
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(DEFAULT_ZENOH_PORT)
        self.port_spin.setToolTip("Zenoh TCP 포트 (기본 7447)")
        bar.addWidget(self.port_spin)
        self.connect_btn = QPushButton("연결")
        self.connect_btn.clicked.connect(self._toggle_connect)
        bar.addWidget(self.connect_btn)
        self.auto_reconnect_cb = QCheckBox("자동 재연결")
        self.auto_reconnect_cb.setChecked(True)
        bar.addWidget(self.auto_reconnect_cb)
        self.status_dot = QLabel("● 연결 안 됨")
        self.status_dot.setStyleSheet("color: #c44; font-weight: bold;")
        bar.addWidget(self.status_dot)
        self.addr_lbl = QLabel("")
        bar.addWidget(self.addr_lbl)
        root.addLayout(bar)

        body = QHBoxLayout()
        root.addLayout(body, stretch=1)

        # 왼쪽: 3D
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setCameraPosition(distance=30)
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.gl_view.addItem(grid)
        # pxMode=True → size는 화면 픽셀 단위. False면 월드(미터) 단위라 size=2가 2m 구로 보임.
        self._scatter = gl.GLScatterPlotItem(
            pos=np.zeros((0, 3), dtype=np.float32),
            color=(0.9, 0.9, 0.9, 1.0),
            size=2,
            pxMode=True,
        )
        self.gl_view.addItem(self._scatter)
        self._path_line = gl.GLLinePlotItem(
            pos=np.zeros((0, 3), dtype=np.float32),
            color=(0.2, 1.0, 0.2, 1.0),
            width=3,
            mode="line_strip",
        )
        self.gl_view.addItem(self._path_line)
        self._pose_scatter = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3), dtype=np.float32),
            color=(1.0, 0.15, 0.15, 1.0),
            size=12,
            pxMode=True,
        )
        self.gl_view.addItem(self._pose_scatter)

        left_box = QGroupBox("3D 맵 (dense_map=칼라 점군 · 경로 · 위치)")
        lv = QVBoxLayout(left_box)
        lv.addWidget(self.gl_view)
        self._map_mode_lbl = QLabel(
            "맵 표시: 연결 후 수신 형식이 여기 표시됩니다. (dense_map은 PointCloud2라 점으로 그립니다)"
        )
        self._map_mode_lbl.setWordWrap(True)
        self._map_mode_lbl.setStyleSheet("color:#aaa; font-size:11px;")
        lv.addWidget(self._map_mode_lbl)
        body.addWidget(left_box, stretch=3)

        # 오른쪽: 카메라 + 상태
        right_col = QVBoxLayout()
        body.addLayout(right_col, stretch=2)

        self._cam_front = self._make_cam_group("전방 카메라 (slam/image/front)")
        self._cam_left = self._make_cam_group("좌측 카메라 (slam/image/left)")
        right_col.addWidget(self._cam_front["box"])
        right_col.addWidget(self._cam_left["box"])

        status_box = QGroupBox("상태")
        sv = QVBoxLayout(status_box)
        self.status_lines = {
            "pos": QLabel("Position : —"),
            "yaw": QLabel("Yaw      : —"),
            "spd": QLabel("Speed    : —"),
            "slam": QLabel("SLAM     : —"),
            "lidar": QLabel("LiDAR    : —"),
            "cpu": QLabel("CPU/MEM  : —"),
        }
        for w in self.status_lines.values():
            w.setFont(QFont("monospace", 10))
            sv.addWidget(w)
        right_col.addWidget(status_box)
        right_col.addStretch()

        # 하단: 명령
        cmd = QHBoxLayout()
        for text, cmd_id in [
            ("E-STOP", "E_STOP"),
            ("Start SLAM", "START_SLAM"),
            ("Stop SLAM", "STOP_SLAM"),
            ("Save Map", "SAVE_MAP"),
            ("Reset", "RESET"),
        ]:
            b = QPushButton(text)
            b.clicked.connect(lambda _, c=cmd_id: self._send(c))
            cmd.addWidget(b)
        self.rec_btn = QPushButton("Record (off)")
        self.rec_btn.clicked.connect(self._toggle_record)
        cmd.addWidget(self.rec_btn)
        root.addLayout(cmd)

        self._reconnect_timer = QTimer(self)
        self._reconnect_timer.setInterval(5000)
        self._reconnect_timer.timeout.connect(self._try_reconnect)

        self._apply_disconnected_ui()

    def _make_cam_group(self, title: str) -> dict:
        box = QGroupBox(title)
        lay = QVBoxLayout(box)
        lbl = QLabel("(no image)")
        lbl.setMinimumSize(640, 360)
        lbl.setMaximumSize(640, 360)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("background:#222; color:#888;")
        lay.addWidget(lbl)
        return {"box": box, "lbl": lbl}

    def _toggle_connect(self) -> None:
        if self._receiver and self._receiver.session:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        ip = self.ip_edit.text().strip()
        if not ip:
            QMessageBox.warning(
                self,
                "연결",
                "로봇 IP 주소를 입력하세요.",
            )
            return
        port = int(self.port_spin.value())
        self._port = port
        if self._receiver:
            self._receiver.disconnect_session()
            self._receiver.deleteLater()
        self._receiver = ZenohReceiver(ip, port)
        self._receiver.pose_received.connect(self._on_pose)
        self._receiver.path_received.connect(self._on_path)
        self._receiver.pointcloud_received.connect(self._on_pointcloud)
        self._receiver.status_received.connect(self._on_status)
        self._receiver.image_front_received.connect(
            lambda b: self._on_jpeg(b, self._cam_front["lbl"])
        )
        self._receiver.image_left_received.connect(
            lambda b: self._on_jpeg(b, self._cam_left["lbl"])
        )
        self._receiver.connection_changed.connect(self._on_connection_changed)
        self._receiver.connection_error.connect(self._on_connection_error)

        ok = self._receiver.connect_session()
        if ok:
            self.connect_btn.setText("연결 해제")
            self.addr_lbl.setText(f"{ip}:{port}")
            self._reconnect_timer.stop()
        else:
            self._apply_disconnected_ui()
            if self.auto_reconnect_cb.isChecked():
                self._reconnect_timer.start()

    def _disconnect(self) -> None:
        self._reconnect_timer.stop()
        if self._receiver:
            self._receiver.disconnect_session()
            self._receiver.deleteLater()
            self._receiver = None
        self._apply_disconnected_ui()

    def _apply_disconnected_ui(self) -> None:
        self.connect_btn.setText("연결")
        self.status_dot.setText("● Disconnected")
        self.status_dot.setStyleSheet("color: #c44; font-weight: bold;")
        self.addr_lbl.setText("")

    def _on_connection_changed(self, connected: bool) -> None:
        if connected:
            self._reconnect_timer.stop()
            self.status_dot.setText("● 연결됨")
            self.status_dot.setStyleSheet("color: #4c4; font-weight: bold;")
            self.connect_btn.setText("연결 해제")
        else:
            self.status_dot.setText("● 연결 안 됨")
            self.status_dot.setStyleSheet("color: #c44; font-weight: bold;")
            self.connect_btn.setText("연결")
            if self.auto_reconnect_cb.isChecked():
                self._reconnect_timer.start()

    def _on_connection_error(self, msg: str) -> None:
        self.status_dot.setToolTip(msg)

    def _try_reconnect(self) -> None:
        if not self.auto_reconnect_cb.isChecked():
            return
        if self._receiver and self._receiver.session:
            return
        ip = self.ip_edit.text().strip()
        if not ip:
            return
        port = int(self.port_spin.value())
        self._port = port
        if self._receiver is None:
            self._receiver = ZenohReceiver(ip, port)
            self._receiver.pose_received.connect(self._on_pose)
            self._receiver.path_received.connect(self._on_path)
            self._receiver.pointcloud_received.connect(self._on_pointcloud)
            self._receiver.status_received.connect(self._on_status)
            self._receiver.image_front_received.connect(
                lambda b: self._on_jpeg(b, self._cam_front["lbl"])
            )
            self._receiver.image_left_received.connect(
                lambda b: self._on_jpeg(b, self._cam_left["lbl"])
            )
            self._receiver.connection_changed.connect(self._on_connection_changed)
            self._receiver.connection_error.connect(self._on_connection_error)
        else:
            self._receiver.robot_ip = ip
            self._receiver.port = port
        ok = self._receiver.connect_session()
        if ok:
            self.addr_lbl.setText(f"{ip}:{self._port}")
            self.connect_btn.setText("연결 해제")
            self._reconnect_timer.stop()

    def _send(self, command: str) -> None:
        if not self._receiver or not self._receiver.session:
            QMessageBox.warning(self, "연결", "Zenoh에 연결되지 않았습니다.")
            return
        self._receiver.send_command(command)

    def _toggle_record(self) -> None:
        if not self._receiver or not self._receiver.session:
            QMessageBox.warning(self, "연결", "Zenoh에 연결되지 않았습니다.")
            return
        if self._record_on:
            self._receiver.send_command("STOP_RECORD")
            self._record_on = False
            self.rec_btn.setText("Record (off)")
        else:
            self._receiver.send_command("START_RECORD")
            self._record_on = True
            self.rec_btn.setText("Record (on)")

    def _on_pose(self, data: dict) -> None:
        self._last_pose = data
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        z = float(data.get("z", 0.0))
        self._pose_scatter.setData(
            pos=np.array([[x, y, z]], dtype=np.float32),
            size=12,
            pxMode=True,
        )
        self._refresh_status_labels()

    def _on_path(self, data: dict) -> None:
        pts = data.get("points") or []
        if not pts:
            self._path_line.setData(pos=np.zeros((0, 3), dtype=np.float32))
            return
        arr = np.asarray(pts, dtype=np.float32)
        if arr.ndim != 2 or arr.shape[1] != 3:
            return
        self._path_line.setData(pos=arr)

    def _on_pointcloud(self, data: dict) -> None:
        """slam/pointcloud: points (N,3); rgb 바이트(여러 포맷) 또는 intensity (N,) float32."""
        raw = data.get("points")
        if not raw:
            self._scatter.setData(
                pos=np.zeros((0, 3), dtype=np.float32),
                color=(0.9, 0.9, 0.9, 1.0),
                size=2,
                pxMode=True,
            )
            self._map_mode_lbl.setText("맵 표시: 포인트 없음")
            return
        points = np.frombuffer(raw, dtype=np.float32).reshape(-1, 3)
        n = len(points)
        fid = data.get("frame_id", "—")

        rgb_raw = data.get("rgb")
        colors: np.ndarray
        pc_size = 2
        if rgb_raw:
            rgba, tag = _rgb_field_to_rgba_float(rgb_raw, n)
            if rgba is not None:
                colors = rgba
                self._map_mode_lbl.setText(
                    f"맵: dense_map 스타일 컬러 ({n}점, frame={fid}) · 디코드: {tag}"
                )
                pc_size = 3
            else:
                colors = np.ones((n, 4), dtype=np.float32)
                colors[:, :3] = 0.85
                self._map_mode_lbl.setText(
                    f"맵: rgb 필드 있으나 디코드 실패 ({n}점) — {tag or '형식 확인'}"
                )
        elif data.get("intensity"):
            intensity = np.frombuffer(data["intensity"], dtype=np.float32)
            m = min(len(intensity), n)
            colors = intensity_to_colormap(intensity[:m])
            if m < n:
                pad = np.ones((n - m, 4), dtype=np.float32)
                pad[:, :3] = 0.85
                colors = np.vstack([colors, pad])
            self._map_mode_lbl.setText(
                f"맵: LiDAR intensity 컬러맵 ({n}점, frame={fid}) — registered cloud"
            )
        else:
            colors = np.ones((n, 4), dtype=np.float32)
            colors[:, :3] = 0.85
            self._map_mode_lbl.setText(
                f"맵: RGB/intensity 없음 → 회색 점만 표시 ({n}점, frame={fid}). "
                "브릿지에서 dense_map의 rgb 필드를 보내는지 확인하세요."
            )

        self._scatter.setData(pos=points, color=colors, size=pc_size, pxMode=True)

    def _on_status(self, data: dict) -> None:
        self._last_status = data
        self._refresh_status_labels()

    def _refresh_status_labels(self) -> None:
        p = self._last_pose or {}
        s = self._last_status or {}
        x, y, z = p.get("x"), p.get("y"), p.get("z")
        if x is not None and y is not None and z is not None:
            self.status_lines["pos"].setText(
                f"Position : x={float(x):.2f} y={float(y):.2f} z={float(z):.2f}"
            )
        else:
            self.status_lines["pos"].setText("Position : —")
        yaw = p.get("yaw")
        if yaw is not None:
            deg = float(yaw) * 180.0 / math.pi
            self.status_lines["yaw"].setText(f"Yaw      : {deg:.1f}°")
        else:
            self.status_lines["yaw"].setText("Yaw      : —")
        vx = p.get("vx")
        if vx is not None:
            self.status_lines["spd"].setText(f"Speed    : {float(vx):.2f} m/s")
        else:
            self.status_lines["spd"].setText("Speed    : —")
        state = s.get("slam_state", "—")
        fc = s.get("frame_count", "—")
        self.status_lines["slam"].setText(f"SLAM     : {state} (Frame #{fc})")
        lz = s.get("lidar_hz")
        iz = s.get("imu_hz")
        if lz is not None:
            extra = f"  IMU {float(iz):.0f} Hz" if iz is not None else ""
            self.status_lines["lidar"].setText(
                f"LiDAR    : {float(lz):.1f} Hz{extra}"
            )
        else:
            self.status_lines["lidar"].setText("LiDAR    : —")
        cpu = s.get("cpu_usage")
        mem = s.get("mem_usage")
        if cpu is not None and mem is not None:
            self.status_lines["cpu"].setText(
                f"CPU/MEM  : {float(cpu):.0f}% / {float(mem):.0f}%"
            )
        else:
            self.status_lines["cpu"].setText("CPU/MEM  : —")

    def _on_jpeg(self, jpeg_bytes: bytes, lbl: QLabel) -> None:
        # cv2는 import 시 QT_QPA_PLATFORM_PLUGIN_PATH를 자기 Qt 플러그인으로 바꿔
        # PyQt5와 충돌하므로 QApplication 생성 이후에만 로드한다.
        import cv2

        if not jpeg_bytes:
            return
        buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            return
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_rgb = np.ascontiguousarray(img_rgb)
        h, w, ch = img_rgb.shape
        qimg = QImage(
            img_rgb.data, w, h, ch * w, QImage.Format_RGB888
        ).copy()
        lbl.setPixmap(
            QPixmap.fromImage(qimg).scaled(
                640, 360, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )

    def closeEvent(self, event) -> None:
        self._reconnect_timer.stop()
        if self._receiver:
            self._receiver.disconnect_session()
        event.accept()


def main() -> int:
    # 셸/다른 패키지가 남긴 경로가 PyQt5 xcb 플러그인 로딩을 가릴 수 있음
    for _k in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_PLUGIN_PATH"):
        os.environ.pop(_k, None)

    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
