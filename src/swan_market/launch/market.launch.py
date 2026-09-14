"""Market scene + existing wheelchair and sensor bridges; no driving commands."""
import os
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('swan_market'))
    wheelchair_share = Path(get_package_share_directory('wheelchair_gazebo'))
    spawn_pose = json.loads((share / 'config/market_layout.json').read_text())['spawn']
    model_file = wheelchair_share / 'models/wheelchair/model.sdf'
    sdf = ET.parse(model_file)
    camera = sdf.find(".//sensor[@type='camera']")
    lidar = sdf.find(".//sensor[@type='gpu_lidar']")
    if camera is None or lidar is None:
        raise RuntimeError(f'Camera and gpu_lidar required in {model_file}')
    camera_topic = camera.findtext('topic')
    lidar_topic = lidar.findtext('topic')
    if not camera_topic or not lidar_topic:
        raise RuntimeError('Sensor topics must be explicit in wheelchair model')
    resource_paths = [str(share / 'models'), str(wheelchair_share / 'models')]
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_paths.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    cmd = ['gz', 'sim', '-r']
    if LaunchConfiguration('headless').perform(context).lower() == 'true':
        cmd += ['-s', '--headless-rendering']
    cmd.append(str(share / 'worlds/market_shopping.sdf'))
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/wheelchair/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/wheelchair/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/wheelchair/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            lidar_topic + '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            camera_topic + '@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[('/model/wheelchair/odometry', '/odom'),
                    ('/model/wheelchair/tf', '/tf'),
                    (lidar_topic, '/scan'), (camera_topic, '/camera')],
        parameters=[{'use_sim_time': True}],
    )
    # ros_gz_sim create waits for the named world's creation service.
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'swan_market', '-file', str(model_file),
                            '-name', 'wheelchair', '-x', str(spawn_pose['x']),
                            '-y', str(spawn_pose['y']), '-z', str(spawn_pose['z']),
                            '-Y', str(spawn_pose['yaw'])])
    actions = []
    if LaunchConfiguration('software_rendering').perform(context).lower() == 'true':
        actions += [SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
                    UnsetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE'),
                    SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')]
    actions += [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resource_paths)),
                ExecuteProcess(cmd=cmd, output='screen'), spawn, bridge]
    if LaunchConfiguration('moving_traffic').perform(context).lower() == 'true':
        actions.append(ExecuteProcess(
            cmd=[sys.executable,str(share/'tools/animate_crosswalk.py'),
                 '--layout',str(share/'config/market_layout.json')],output='screen'))
    # Preserve both transforms; remote versions may tilt the LiDAR mount.
    lidar_link = next(link for link in sdf.findall('.//model/link') if lidar in list(link))
    sensor_frame = lidar.findtext('gz_frame_id') or f'wheelchair/{lidar_link.get("name")}/{lidar.get("name")}'
    for name, parent, child, pose in [
        ('market_lidar_mount_tf', 'base_link', 'market_lidar_mount', lidar_link.findtext('pose', '0 0 0 0 0 0')),
        ('market_lidar_sensor_tf', 'market_lidar_mount', sensor_frame, lidar.findtext('pose', '0 0 0 0 0 0')),
    ]:
        x, y, z, roll, pitch, yaw = pose.split()
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                            arguments=['--x', x, '--y', y, '--z', z, '--roll', roll,
                                       '--pitch', pitch, '--yaw', yaw,
                                       '--frame-id', parent, '--child-frame-id', child],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('moving_traffic', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('software_rendering', default_value='false',
                             description='Use Mesa llvmpipe and X11 for UTM VM rendering'),
        OpaqueFunction(function=setup),
    ])
