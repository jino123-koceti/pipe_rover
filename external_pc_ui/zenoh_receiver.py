"""Zenoh 구독/발행을 Qt 시그널로 연결 (콜백은 Zenoh 스레드에서 호출됨)."""

from __future__ import annotations

import msgpack
from PyQt5.QtCore import QObject, pyqtSignal


def _make_msgpack_handler(signal):
    def handler(sample):
        try:
            raw = sample.payload.to_bytes()
            data = msgpack.unpackb(raw, raw=False)
            signal.emit(data)
        except Exception:
            pass

    return handler


def _make_bytes_handler(signal):
    def handler(sample):
        try:
            signal.emit(sample.payload.to_bytes())
        except Exception:
            pass

    return handler


class ZenohReceiver(QObject):
    pose_received = pyqtSignal(dict)
    path_received = pyqtSignal(dict)
    pointcloud_received = pyqtSignal(dict)
    status_received = pyqtSignal(dict)
    image_front_received = pyqtSignal(bytes)
    image_left_received = pyqtSignal(bytes)
    connection_changed = pyqtSignal(bool)
    connection_error = pyqtSignal(str)

    def __init__(self, robot_ip: str, port: int = 7447, parent=None):
        super().__init__(parent)
        self.robot_ip = robot_ip
        self.port = port
        self.session = None
        self._subscribers = []

    def connect_session(self) -> bool:
        import zenoh

        self.disconnect_session()
        try:
            config = zenoh.Config()
            config.insert_json5("scouting/multicast/enabled", "false")
            config.insert_json5(
                "connect/endpoints",
                f'["tcp/{self.robot_ip}:{self.port}"]',
            )
            self.session = zenoh.open(config)
        except Exception as e:
            self.session = None
            self.connection_error.emit(str(e))
            self.connection_changed.emit(False)
            return False

        for key, signal in [
            ("slam/pose", self.pose_received),
            ("slam/path", self.path_received),
            ("slam/pointcloud", self.pointcloud_received),
            ("slam/status", self.status_received),
        ]:
            sub = self.session.declare_subscriber(
                key, _make_msgpack_handler(signal)
            )
            self._subscribers.append(sub)

        for key, signal in [
            ("slam/image/front", self.image_front_received),
            ("slam/image/left", self.image_left_received),
        ]:
            sub = self.session.declare_subscriber(
                key, _make_bytes_handler(signal)
            )
            self._subscribers.append(sub)

        self.connection_changed.emit(True)
        return True

    def send_command(self, command: str) -> None:
        if self.session:
            self.session.put("slam/command", command.encode("utf-8"))

    def disconnect_session(self) -> None:
        self._subscribers.clear()
        if self.session:
            try:
                self.session.close()
            except Exception:
                pass
            self.session = None
        self.connection_changed.emit(False)
