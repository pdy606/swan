"""Derived camera + unchanged SWAN controllers + opt-in MBRA driver. No Teleop."""
import os
import json
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    from wheelchair_ai_driver.derived_world import generate
    root = Path(LaunchConfiguration('package_root').perform(context)).resolve()
    src = Path(LaunchConfiguration('swan_source_root').perform(context)).resolve()
    spawn_text = LaunchConfiguration('test_spawn').perform(context)
    spawn = json.loads(spawn_text) if spawn_text else None
    world = generate(root, src, test_spawn=spawn,
                     test_wall=LaunchConfiguration('test_wall').perform(context).lower() == 'true',
                     test_obstacle_width=float(LaunchConfiguration('test_obstacle_width').perform(context)))
    evaluation_dir = LaunchConfiguration('evaluation_dir').perform(context)
    if evaluation_dir:
        from wheelchair_ai_driver.evaluation_world import instrument
        world = instrument(world, root, evaluation_dir)
    nav = src/'wheelchair_navigation'
    gazebo = src/'wheelchair_gazebo'
    with_nav = LaunchConfiguration('with_nav2').perform(context).lower() == 'true'
    assist = LaunchConfiguration('assistance').perform(context).lower() == 'true'
    with_vision = LaunchConfiguration('with_vision').perform(context).lower() == 'true'
    if assist and not with_nav:
        raise ValueError('assistance:=true requires with_nav2:=true')
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    gui_config = LaunchConfiguration('gui_config').perform(context)
    # Launch GUI separately so gz's combined-mode children cannot outlive ROS launch.
    gz_arguments = f'-s -r {world}'
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(filter(None, [
            str(gazebo/'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_share/'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_arguments, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ai_driving_bridge',
             output='screen', arguments=[
                 '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/camera_driver@sensor_msgs/msg/Image[gz.msgs.Image',
                 '/model/wheelchair/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/model/wheelchair/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                 '/model/wheelchair/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                 '/model/wheelchair/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/model/wheelchair/front_drive/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                 '/model/wheelchair/front_steering/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                 '/model/wheelchair/wheel_retract/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                 '/model/wheelchair/slide/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                 '/model/wheelchair/camera_fold/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
             ], remappings=[('/model/wheelchair/odometry', '/odom'),
                            ('/model/wheelchair/tf', '/tf'), ('/model/wheelchair/scan', '/scan')]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='ai_lidar_tf',
             arguments=['--x', '0.932', '--y', '0', '--z', '0.12', '--yaw', '0',
                        '--frame-id', 'base_link', '--child-frame-id', 'wheelchair/lidar_link/lidar_sensor']),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain', name='scan_to_scan_filter_chain',
             parameters=[str(nav/'config/scan_filter.yaml'), {'use_sim_time': True}],
             remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')]),
    ]
    if gui:
        command=['gz','sim','-g']
        if gui_config:
            command.extend(['--gui-config',gui_config])
        actions.append(ExecuteProcess(cmd=command,name='ai_gazebo_gui',output='screen',
            on_exit=[EmitEvent(event=Shutdown(reason='Gazebo GUI closed'))]))
    # Running the original Python sources avoids building or changing any existing package.
    for script in [gazebo/'scripts/swan_drive_controller.py', nav/'scripts/nav_avoidance_node.py',
                   src/'swan_pipeline/swan_pipeline/situation_node.py']:
        actions.append(ExecuteProcess(cmd=[sys.executable, str(script), '--ros-args', '-p',
                                           'use_sim_time:=true'], output='screen'))
    if with_nav:
        for package, executable in [('nav2_controller', 'controller_server'),
                                    ('nav2_planner', 'planner_server'),
                                    ('nav2_bt_navigator', 'bt_navigator'),
                                    ('nav2_behaviors', 'behavior_server')]:
            actions.append(Node(package=package, executable=executable, name=executable,
                parameters=[str(nav/'config/nav2_params.yaml'), {'use_sim_time': True}],
                remappings=[('cmd_vel', '/cmd_vel_nav')] if executable in ('controller_server', 'behavior_server') else [],
                output='screen'))
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='ai_nav2_lifecycle_manager', parameters=[{'use_sim_time': True, 'autostart': True,
              'node_names': ['controller_server', 'planner_server', 'bt_navigator', 'behavior_server']}]))
    if with_vision:
        # Optional original YOLO executable; no alternate detector, remapping or filtering.
        get_package_share_directory('wheelchair_vision')
        actions.append(Node(package='wheelchair_vision', executable='yolo_node',
                            parameters=[{'use_sim_time': True}], output='screen'))
    overrides = {'package_root': str(root), 'use_sim_time': True,
            'assistance_enabled': assist,
            'max_speed_mps': float(LaunchConfiguration('max_speed').perform(context))}
    route_text = LaunchConfiguration('route').perform(context)
    if route_text:
        overrides['route_world_xy'] = [float(v) for v in json.loads(route_text)]
    actions.append(Node(package='wheelchair_ai_driver', executable='mbra_driver', name='mbra_driver',
        parameters=[str(root/'config/driving.yaml'), overrides], output='screen'))
    return actions


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('package_root', default_value=str(root)),
        DeclareLaunchArgument('swan_source_root', default_value=str(root.parent)),
        DeclareLaunchArgument('evaluation_dir', default_value='', description='Optional evaluation-only solver contact output'),
        DeclareLaunchArgument('with_nav2', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('gui_config', default_value=''),
        DeclareLaunchArgument('with_vision', default_value='false',
                             description='Run unchanged YOLO from an already installed original package'),
        DeclareLaunchArgument('assistance', default_value='false'),
        DeclareLaunchArgument('max_speed', default_value='0.30'),
        DeclareLaunchArgument('route', default_value='', description='Optional flat JSON XY list'),
        DeclareLaunchArgument('test_spawn', default_value='', description='TEST ONLY: JSON [x,y,z,roll,pitch,yaw]'),
        DeclareLaunchArgument('test_wall', default_value='false', description='TEST ONLY: static wall ahead of test spawn'),
        DeclareLaunchArgument('test_obstacle_width', default_value='8.0'),
        OpaqueFunction(function=setup),
    ])
