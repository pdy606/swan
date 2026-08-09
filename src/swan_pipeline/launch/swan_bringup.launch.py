"""
SWAN 통합 브링업 (단일 진입점) — 옛 merged_bringup/camera/overlay/pipeline 4개를 하나로 통합.

역할: 팀 커리큘럼 월드 위에 내 주행 스택(인지→퓨전→판단→제어)을 얹어 실행.

  simulation/worlds/<world>.world  ─(gz sim)→  월드 로드 (팀 소유, 불변)
  swan_pipeline/models/wheelchair  ─(spawn)→  휠체어(LiDAR+카메라) 얹기
  /scan  → lidar_perception → ┐
  /image → camera_perception → fusion → assist → control → /model/wheelchair/cmd_vel

사용:
  ros2 launch swan_pipeline swan_bringup.launch.py world:=level1_basic
  옵션: use_camera:=true  use_teleop:=false  spawn_x:=-3.0 spawn_y:=-13.0 spawn_yaw:=1.5708
        worlds_dir:=<팀 simulation/worlds 절대경로>   (미지정 시 SWAN_WORLDS_DIR 환경변수)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CMD_VEL = '/model/wheelchair/cmd_vel'


def _worlds_dir(context):
    """월드 폴더 결정: 인자 > 환경변수 SWAN_WORLDS_DIR."""
    d = LaunchConfiguration('worlds_dir').perform(context)
    return d or os.environ.get('SWAN_WORLDS_DIR', '')


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('swan_pipeline')
    st = {'use_sim_time': True}
    use_camera = LaunchConfiguration('use_camera')
    use_teleop = LaunchConfiguration('use_teleop')

    world_name = LaunchConfiguration('world').perform(context)
    world_path = os.path.join(_worlds_dir(context), world_name + '.world')
    wheelchair_sdf = os.path.join(pkg, 'models', 'wheelchair', 'model.sdf')
    sx = LaunchConfiguration('spawn_x').perform(context)
    sy = LaunchConfiguration('spawn_y').perform(context)
    syaw = LaunchConfiguration('spawn_yaw').perform(context)

    # 1) 팀 월드 로드
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world_path], output='screen')

    # 2) 휠체어(내 모델) 스폰
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-file', wheelchair_sdf, '-name', 'wheelchair',
                            '-x', sx, '-y', sy, '-z', '0.05', '-Y', syaw])

    # 3) gz↔ROS 브리지
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/wheelchair/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/wheelchair/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            CMD_VEL + '@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/wheelchair/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[('/model/wheelchair/scan', '/scan'),
                    ('/model/wheelchair/odometry', '/odom')])

    # 4) 파이프라인 (센서 뜬 뒤 시작)
    pipeline = TimerAction(period=6.0, actions=[
        Node(package='swan_pipeline', executable='lidar_perception_node', output='screen',
             parameters=[st, {'out_topic': '/swan/lidar_detections'}]),
        Node(package='swan_pipeline', executable='camera_perception_node', output='screen',
             parameters=[st, {'image_topic': '/wheelchair/camera/image'}],
             condition=IfCondition(use_camera)),
        Node(package='swan_pipeline', executable='fusion_node', output='screen',
             parameters=[st], condition=IfCondition(use_camera)),
        # 카메라 끄면 LiDAR 검출을 바로 판단으로 (fusion 우회)
        Node(package='swan_pipeline', executable='lidar_perception_node', output='screen',
             parameters=[st, {'out_topic': '/swan/detections'}], name='lidar_direct',
             condition=UnlessCondition(use_camera)),
        Node(package='swan_pipeline', executable='assist_node', output='screen', parameters=[st]),
        Node(package='swan_pipeline', executable='control_node', output='screen',
             parameters=[st, {'output_topic': CMD_VEL}]),
        Node(package='swan_pipeline', executable='virtual_user', output='screen',
             parameters=[st], condition=UnlessCondition(use_teleop)),
    ])

    return [gz, TimerAction(period=3.0, actions=[spawn]), bridge, pipeline]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='level1_basic',
                              description='팀 월드 이름(확장자 제외): level1_basic~level4_unseen_eval, layout_*'),
        DeclareLaunchArgument('worlds_dir', default_value='',
                              description='팀 simulation/worlds 절대경로 (미지정 시 $SWAN_WORLDS_DIR)'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='-3.0'),   # 인도 중앙(도로 왼쪽)
        DeclareLaunchArgument('spawn_y', default_value='-13.0'),  # 30m 도로 시작부
        DeclareLaunchArgument('spawn_yaw', default_value='1.5708'),  # +Y 방향
        OpaqueFunction(function=launch_setup),
    ])
