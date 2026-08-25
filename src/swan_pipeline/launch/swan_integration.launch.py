"""
SWAN 통합 커넥터 — 내 파트를 팀에 꽂는 최소 구성. **노드 3개.**

┌ 팀이 제공 ────────────────────────────────────────────┐
│  sim + /scan_filtered + /odom          (다영·규원)       │
│  /yolo/detected_objects                (다영 yolo_node)  │
│  Nav2 + nav_avoidance (cmd_vel arbiter)(규원)           │
└─────────────────────────────────────────────────────────┘
┌ 내가 띄우는 것 (이 런치, 3개) ────────────────────────────┐
│  fusion_node    /scan_filtered + /yolo → /swan/detections │  ← 퍼셉션 코어(lidar·yolo 흡수)
│  situation_node /swan/detections → /driving_situation="C" │  ← C: 회피
│  signal_node    /yolo(신호등) → /driving_situation="B"    │  ← B: 의미기반(빨강=정지)
└─────────────────────────────────────────────────────────┘
              A=정상 / B=신호(의미) / C=회피

사용:
  ros2 launch swan_pipeline swan_integration.launch.py
  (팀 sim/scan_filter/YOLO/Nav2 를 먼저 띄운 상태 전제)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ST = {'use_sim_time': True}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('trigger_dist', default_value='1.5'),
        # 퍼셉션 코어 (lidar 클러스터링 + yolo 클래스결합 흡수)
        Node(package='swan_pipeline', executable='fusion_node', output='screen',
             parameters=[ST]),
        # C: 회피 판단
        Node(package='swan_pipeline', executable='situation_node', output='screen',
             parameters=[ST, {'trigger_dist': LaunchConfiguration('trigger_dist')}]),
        # B: 신호등 등 의미기반 행동
        Node(package='swan_pipeline', executable='signal_node', output='screen',
             parameters=[ST]),
    ])
