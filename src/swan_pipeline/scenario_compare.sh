#!/usr/bin/env bash
# 상황 판단 검증: 같은 위치 장애물을 '사람' vs '박스'로 → 행동 차이 CSV 기록
#  빈 월드에 로봇만 + GT인지(클래스 지정) + assist + control + 로거
source /opt/ros/jazzy/setup.bash; source ~/merged_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
SHARE=~/merged_ws/install/swan_description/share/swan_description 2>/dev/null
# swan_description 은 병합에 없을 수 있음 → 팀 휠체어 대신 우리 URDF 재사용 or 팀 월드
DESC=$(ros2 pkg prefix swan_description 2>/dev/null)/share/swan_description
URDF=$DESC/urdf/swan_wheelchair.urdf
WORLD=$DESC/worlds/swan_world.sdf   # 빈 월드 (장애물은 GT로 가상)

kill_all(){ pkill -9 -f 'gz sim'; pkill -9 -f parameter_bridge; pkill -9 -f '[g]t_perception'
            pkill -9 -f '[a]ssist_node'; pkill -9 -f '[c]ontrol_node'; pkill -9 -f '[v]irtual_user'
            pkill -9 -f '[s]cenario_logger'; sleep 2; }
trap kill_all EXIT

run_case(){
  CLS=$1; NAME=$2; CSV=$3
  echo ">>> [$NAME] (class_id=$CLS) → $CSV"
  kill_all
  gz sim -s -r "$WORLD" > /tmp/gz.log 2>&1 & sleep 6
  ros2 run ros_gz_sim create -file "$URDF" -z 0.19 -name swan_bot > /dev/null 2>&1
  ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
    /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry > /dev/null 2>&1 & sleep 2
  # GT 인지: 장애물 (5, 0) 정면, 클래스 지정
  ros2 run swan_pipeline gt_perception_node --ros-args \
    -p obs_x:=5.0 -p obs_y:=0.0 -p obs_class_id:=$CLS -p obs_class_name:=$NAME > /dev/null 2>&1 &
  ros2 run swan_pipeline assist_node > /tmp/a_$NAME.log 2>&1 &
  ros2 run swan_pipeline control_node --ros-args -p output_topic:=/cmd_vel > /dev/null 2>&1 &
  ros2 run swan_pipeline virtual_user > /dev/null 2>&1 &
  ros2 run swan_pipeline scenario_logger --ros-args -p output:=$CSV -p cmd_topic:=/cmd_vel > /dev/null 2>&1 &
  sleep 14
  echo "    샘플 $(wc -l < $CSV)줄 / 상태전환:"; grep "상태 전환" /tmp/a_$NAME.log | sed 's/.*상태 전환/      /'
  kill_all
}

run_case 3 person /tmp/scenario_person.csv
run_case 0 box    /tmp/scenario_box.csv
echo "완료: /tmp/scenario_person.csv, /tmp/scenario_box.csv"
