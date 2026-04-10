#!/bin/bash
# VILL-SLAM UI 실행 스크립트
# Qt 플러그인 충돌 해결

# OpenCV Qt 플러그인 비활성화 (필요시)
CV2_QT_PATH="$HOME/.local/lib/python3.10/site-packages/cv2/qt"
if [ -d "$CV2_QT_PATH" ]; then
    echo "Disabling OpenCV Qt plugins temporarily..."
    mv "$CV2_QT_PATH" "${CV2_QT_PATH}_disabled" 2>/dev/null
fi

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Qt 플랫폼 플러그인 경로 설정
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms

# DISPLAY 확인
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

echo "Starting VILL-SLAM Monitor UI..."
echo "DISPLAY=$DISPLAY"

# UI 실행
ros2 run vill_slam_ui vill_slam_monitor.py

# OpenCV Qt 플러그인 복구
if [ -d "${CV2_QT_PATH}_disabled" ]; then
    mv "${CV2_QT_PATH}_disabled" "$CV2_QT_PATH" 2>/dev/null
fi
