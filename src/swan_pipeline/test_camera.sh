#!/usr/bin/env bash
# 카메라 스택 검증: 팀월드 + 오버레이(차/보행자) + 카메라인지 + LiDAR인지 + 퓨전
source /opt/ros/jazzy/setup.bash; source ~/merged_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/merged_ws/install/wheelchair_gazebo/share/wheelchair_gazebo/models
WORLD=~/merged_ws/install/wheelchair_gazebo/share/wheelchair_gazebo/worlds/wheelchair_world.sdf
CAR=~/merged_ws/install/swan_pipeline/share/swan_pipeline/models/illegal_car/model.sdf
PED=~/merged_ws/install/swan_pipeline/share/swan_pipeline/models/moving_pedestrian/model.sdf
kill_all(){ pkill -9 -f 'gz sim'; pkill -9 -f parameter_bridge; pkill -9 -f perception
            pkill -9 -f fusion; pkill -9 -f assist_node; pkill -9 -f control_node; sleep 2; }
trap kill_all EXIT; kill_all

gz sim -s -r --headless-rendering "$WORLD" > /tmp/gz.log 2>&1 & sleep 9
# 카메라 + scan + odom + cmd 브리지
ros2 run ros_gz_bridge parameter_bridge \
  /wheelchair/camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /model/wheelchair/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  --ros-args -r /model/wheelchair/scan:=/scan > /dev/null 2>&1 & sleep 3
# 앞에 차·보행자 스폰 (카메라에 잡히게 x=4~6)
ros2 run ros_gz_sim create -file "$CAR" -name illegal_car -x 5 -y 0.3 -z 0.7 > /dev/null 2>&1
ros2 run ros_gz_sim create -file "$PED" -name moving_pedestrian -x 4 -y -1 -z 0.85 > /dev/null 2>&1
sleep 3

echo "=== 1) 카메라 /image 나오나 ==="
CNT=$(timeout 6 ros2 topic echo /wheelchair/camera/image --once 2>/dev/null | grep -c "encoding")
echo "  image 메시지: $([ "$CNT" -gt 0 ] && echo ✅나옴 || echo ❌없음)"
timeout 4 ros2 topic echo /wheelchair/camera/image --once 2>/dev/null | grep -E "width:|height:|encoding:" | head -3

echo ""
echo "=== 2) camera_perception 인식 결과 ==="
ros2 run swan_pipeline camera_perception_node > /tmp/cam.log 2>&1 &
sleep 3
timeout 5 ros2 topic echo /swan/cam_detections --once 2>/dev/null | grep -E "class_name|distance_m|angle_rad" | head -12 || echo "  검출 없음"

echo ""
echo "=== 3) LiDAR 인지 + 퓨전 ==="
ros2 run swan_pipeline lidar_perception_node --ros-args -p out_topic:=/swan/lidar_detections > /dev/null 2>&1 &
ros2 run swan_pipeline fusion_node > /dev/null 2>&1 &
sleep 3
echo "  퓨전 결과 (거리=LiDAR, 클래스=카메라):"
timeout 5 ros2 topic echo /swan/detections --once 2>/dev/null | grep -E "class_name|distance_m" | head -8 || echo "  없음"
echo "CAM_DONE"
kill_all
