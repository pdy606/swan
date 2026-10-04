import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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
            # ROS 2 -> Gazebo : SWAN drive wheel
            '/model/wheelchair/front_drive/cmd_vel'
            '@std_msgs/msg/Float64'
            '@gz.msgs.Double',

            # ROS 2 -> Gazebo : SWAN steering
            '/model/wheelchair/front_steering/cmd_pos'
            '@std_msgs/msg/Float64'
            '@gz.msgs.Double',

            # ROS 2 -> Gazebo : stow actuators (one-way command bridges).
            # swan_drive_controller.py drives these from
            # /model/wheelchair/fold/cmd_pos (0.0 deploy / non-zero stow).
            '/model/wheelchair/wheel_retract/cmd_pos'
            '@std_msgs/msg/Float64'
            ']gz.msgs.Double',

            '/model/wheelchair/slide/cmd_pos'
            '@std_msgs/msg/Float64'
            ']gz.msgs.Double',

            '/model/wheelchair/camera_fold/cmd_pos'
            '@std_msgs/msg/Float64'
            ']gz.msgs.Double',

            # Gazebo -> ROS 2 odometry
            '/model/wheelchair/odometry'
            '@nav_msgs/msg/Odometry'
            '[gz.msgs.Odometry',

            # Gazebo -> ROS 2 TF
            '/model/wheelchair/tf'
            '@tf2_msgs/msg/TFMessage'
            '[gz.msgs.Pose_V',

            # Gazebo -> ROS 2 LiDAR
            '/model/wheelchair/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            # Gazebo -> ROS 2 Camera
            '/camera'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            # Gazebo -> ROS 2 : SWAN safety sensors
            '/model/wheelchair/imu'
            '@sensor_msgs/msg/Imu'
            '[gz.msgs.IMU',

            '/model/wheelchair/cliff_scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            '/model/wheelchair/rear_scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            # Gazebo -> ROS 2 simulation clock
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
            ('/model/wheelchair/imu', '/swan/imu'),
            ('/model/wheelchair/cliff_scan', '/swan/cliff_scan'),
            ('/model/wheelchair/rear_scan', '/swan/rear_scan'),
        ]
    )

    # ----------------------------------------------------------------
    # Contact sensors (measurement only: counts real collisions).
    # This Gazebo version ignores <topic> for contact sensors and
    # publishes on a world-scoped name, so the bridge needs the world
    # name: pass world_name:=<name> when using a different world file.
    # ----------------------------------------------------------------

    world_name = LaunchConfiguration('world_name')
    contact_links = [
        ('base_link', 'body_contact', 'body'),
        ('swan_slide_link', 'kit_body_contact', 'kit'),
        ('lidar_link', 'lidar_contact', 'lidar'),
    ]

    contact_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheelchair_contact_bridge',
        output='screen',
        arguments=[
            [
                '/world/', world_name,
                f'/model/wheelchair/link/{link}/sensor/{sensor}/contact'
                '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            ]
            for link, sensor, _ in contact_links
        ],
        remappings=[
            (
                [
                    '/world/', world_name,
                    f'/model/wheelchair/link/{link}/sensor/{sensor}/contact',
                ],
                f'/swan/contacts/{short}',
            )
            for link, sensor, short in contact_links
        ],
    )

    # Static frames of the safety sensors (kit extended)
    sensor_static_tfs = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            output='screen',
            arguments=[
                '--x', x, '--y', '0.0', '--z', z,
                '--roll', '0.0', '--pitch', pitch, '--yaw', yaw,
                '--frame-id', 'base_link',
                '--child-frame-id', frame,
            ],
        )
        for name, x, z, pitch, yaw, frame in (
            ('rear_sensor_static_tf', '-0.352', '0.24', '0.0', '3.14159265',
             'wheelchair/base_link/rear_sensor'),
            ('cliff_sensor_static_tf', '1.004', '0.134', '0.7', '0.0',
             'wheelchair/lidar_link/cliff_sensor'),
            ('imu_static_tf', '0.0', '0.0', '0.0', '0.0',
             'wheelchair/base_link/imu_sensor'),
        )
    ]

    swan_drive_controller = Node(
        package='wheelchair_gazebo',
        executable='swan_drive_controller.py',
        name='swan_drive_controller',
        output='screen',
        # stow 시퀀스 타이머가 시뮬레이션 시간으로 돌도록
        parameters=[{'use_sim_time': True}],
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=[
            '--x', '0.932',
            '--y', '0.0',
            '--z', '0.09',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'wheelchair/lidar_link/lidar_sensor',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='swan_test_world',
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_path
        ),

        gazebo_launch,
        bridge,
        contact_bridge,
        lidar_static_tf,
        *sensor_static_tfs,
        swan_drive_controller,
    ])