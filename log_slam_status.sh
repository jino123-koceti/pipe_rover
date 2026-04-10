#!/bin/bash
# SLAM 실시간 모니터링 스크립트 (1분간 위치 기록)
source /opt/ros/humble/setup.bash
source /home/test/ros2_ws/install/setup.bash

LOG_DIR="/home/test/ros2_ws/slam_logs"
mkdir -p $LOG_DIR
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/slam_status_${TIMESTAMP}.log"

echo "=== SLAM Status Log: $(date) ===" > $LOG_FILE
echo "Recording for 60 seconds..." | tee -a $LOG_FILE

# 1. 토픽 목록
echo -e "\n=== Active Topics ===" >> $LOG_FILE
ros2 topic list | grep -E "ouster|zed|Odometry|cloud|path|Laser" >> $LOG_FILE

# 2. 1분간 2초 간격으로 위치 기록
echo -e "\n=== Position Log (2s interval, 60s) ===" >> $LOG_FILE
END=$((SECONDS+60))
COUNT=0
while [ $SECONDS -lt $END ]; do
    POS=$(timeout 2 ros2 topic echo /Odometry --once --field pose.pose.position 2>/dev/null)
    if [ -n "$POS" ]; then
        X=$(echo "$POS" | grep "x:" | awk '{print $2}')
        Y=$(echo "$POS" | grep "y:" | awk '{print $2}')
        Z=$(echo "$POS" | grep "z:" | awk '{print $2}')
        COUNT=$((COUNT+1))
        echo "[$COUNT] t=$(date +%H:%M:%S) x=$X y=$Y z=$Z" | tee -a $LOG_FILE
    else
        COUNT=$((COUNT+1))
        echo "[$COUNT] t=$(date +%H:%M:%S) NO DATA" | tee -a $LOG_FILE
    fi
    sleep 1
done

echo -e "\n=== Final Odometry ===" >> $LOG_FILE
timeout 3 ros2 topic echo /Odometry --once 2>&1 >> $LOG_FILE

echo -e "\n=== Log saved to: $LOG_FILE ==="
echo "Log saved to: $LOG_FILE"
