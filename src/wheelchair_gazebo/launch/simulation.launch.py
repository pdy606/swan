import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wheelchair_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'wheelchair_world.sdf'
    )

    models_path = os.path.join(
        pkg_share,
        'models'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheelchair_bridge',
        output='screen',
        arguments=[
            # ROS 2 <-> Gazebo : 앞바퀴 구동
            '/model/wheelchair/front_drive/cmd_vel'
            '@std_msgs/msg/Float64'
            '@gz.msgs.Double',

            # ROS 2 <-> Gazebo : 앞바퀴 조향
            '/model/wheelchair/front_steering/cmd_pos'
            '@std_msgs/msg/Float64'
            '@gz.msgs.Double',

            # ROS 2 -> Gazebo : 바퀴 수납
            '/model/wheelchair/wheel_retract/cmd_pos'
            '@std_msgs/msg/Float64'
            ']gz.msgs.Double',

            # ROS 2 -> Gazebo : 키트 몸체 슬라이드
            '/model/wheelchair/slide/cmd_pos'
            '@std_msgs/msg/Float64'
            ']gz.msgs.Double',

            # ROS 2 -> Gazebo : 카메라 접기
            '/model/wheelchair/camera_fold/cmd_pos'
            '@std_msgs/msg/Float64'
            ']gz.msgs.Double',

            # /model/wheelchair/fold/cmd_pos는 ROS 내부 명령.
            # 컨트롤러가 위의 세 관절 명령으로 변환하므로
            # 이 토픽 자체는 Gazebo에 브리지하지 않는다.

            # Gazebo -> ROS 2 : odometry
            '/model/wheelchair/odometry'
            '@nav_msgs/msg/Odometry'
            '[gz.msgs.Odometry',

            # Gazebo -> ROS 2 : TF
            '/model/wheelchair/tf'
            '@tf2_msgs/msg/TFMessage'
            '[gz.msgs.Pose_V',

            # Gazebo -> ROS 2 : LiDAR
            '/model/wheelchair/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            # Gazebo -> ROS 2 : camera
            '/camera'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            # Gazebo -> ROS 2 : simulation clock
            '/clock'
            '@rosgraph_msgs/msg/Clock'
            '[gz.msgs.Clock',
        ],
        remappings=[
            (
                '/model/wheelchair/odometry',
                '/odom'
            ),
            (
                '/model/wheelchair/tf',
                '/tf'
            ),
            (
                '/model/wheelchair/scan',
                '/scan'
            ),
        ]
    )

    swan_drive_controller = Node(
        package='wheelchair_gazebo',
        executable='swan_drive_controller.py',
        name='swan_drive_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 키트가 완전히 펼쳐진 상태의 LiDAR 센서 위치.
    # 링크 높이 0.09 + 센서 내부 오프셋 0.03 = 0.12
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=[
            '--x', '0.932',
            '--y', '0.0',
            '--z', '0.12',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'wheelchair/lidar_link/lidar_sensor',
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_path
        ),

        gazebo_launch,
        bridge,
        lidar_static_tf,
        swan_drive_controller,
    ])