#!/usr/bin/env bash
# 더미 눈 카메라 화면 보기: Gazebo→ROS 브리지 + 화면을 한 번에 실행
# 사용법: 시뮬레이션이 켜진 상태에서  ./tools/dummy/eye_view.sh
# 종료: 화면 창을 닫거나 Ctrl+C (브리지도 같이 종료됨)
set -e
source /opt/ros/jazzy/setup.bash
TOPIC=/dummy/eye/image

if ! gz topic -l 2>/dev/null | grep -qx "$TOPIC"; then
  echo "[eye_view] Gazebo에 $TOPIC 토픽이 없습니다."
  echo "  1) 시뮬레이션이 켜져 있는지"
  echo "  2) python3 tools/dummy/add_dummy.py 로 더미를 장착하고 다시 빌드했는지 확인하세요."
  exit 1
fi

ros2 run ros_gz_bridge parameter_bridge "${TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image" \
  --ros-args -r __node:=dummy_eye_bridge >/dev/null 2>&1 &
BRIDGE_PID=$!
trap 'kill $BRIDGE_PID 2>/dev/null; wait $BRIDGE_PID 2>/dev/null; echo "[eye_view] 종료"' EXIT INT TERM

echo "[eye_view] 브리지 실행 (pid $BRIDGE_PID), 화면 여는 중..."
ros2 run image_tools showimage --ros-args -r image:=$TOPIC -r __node:=dummy_eye_view --log-level warn
