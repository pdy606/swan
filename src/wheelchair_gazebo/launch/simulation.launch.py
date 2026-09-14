"""Shared Gazebo entry point: select a world without changing robot topics."""
import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_support_spec = importlib.util.spec_from_file_location('wheelchair_world_support', Path(__file__).with_name('world_support.py'))
_support = importlib.util.module_from_spec(_support_spec)
_support_spec.loader.exec_module(_support)


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    spec = _support.resolve_world(value('world'), value('world_package'), get_package_share_directory)
    pose = _support.spawn_pose(spec, {key:value('spawn_' + key) for key in ('x','y','z','yaw')})
    robot_share, share = spec['robot_share'], spec['share']
    scenario_package = spec['options'].get('scenario_package')
    scenario_share = Path(get_package_share_directory(scenario_package)) if scenario_package else share
    resources = list(dict.fromkeys([str(share / 'models'), str(robot_share / 'models'),
                                    str(scenario_share / 'models'), str(spec['path'].parent)]))
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resources.append(os.environ['GZ_SIM_RESOURCE_PATH'])
    actions = []
    if value('software_rendering').lower() == 'true':
        actions += [SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
                    UnsetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE'),
                    SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')]
    cmd = ['gz', 'sim', '-r']
    if value('headless').lower() == 'true':
        cmd += ['-s', '--headless-rendering']
    actions += [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.pathsep.join(resources)),
                ExecuteProcess(cmd=cmd + [str(spec['path'])], output='screen')]
    if pose is not None:
        actions.append(Node(package='ros_gz_sim', executable='create', output='screen',
            arguments=['-world', spec['name'], '-file', str(robot_share / 'models/wheelchair/model.sdf'),
                       '-name', 'wheelchair', '-x', str(pose['x']), '-y', str(pose['y']),
                       '-z', str(pose['z']), '-Y', str(pose['yaw'])]))
    arguments = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/model/wheelchair/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 '/model/wheelchair/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/model/wheelchair/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
    remappings = [('/model/wheelchair/odometry','/odom'), ('/model/wheelchair/tf','/tf')]
    # Read each branch's actual camera/LiDAR topic, rather than hard-coding one model version.
    for kind, ros_type, gz_type, topic in [('camera','Image','Image','/camera'), ('gpu_lidar','LaserScan','LaserScan','/scan')]:
        sensor = spec['robot'].find(f".//sensor[@type='{kind}']")
        if sensor is None:
            continue
        source = sensor.findtext('topic')
        if not source:
            raise ValueError(f'{kind} sensor requires an explicit topic')
        arguments.append(f'{source}@sensor_msgs/msg/{ros_type}[gz.msgs.{gz_type}')
        remappings.append((source, topic))
        if kind == 'gpu_lidar':
            link = next(link for link in spec['robot'].findall('link') if sensor in list(link))
            frame = sensor.findtext('gz_frame_id') or f"wheelchair/{link.get('name')}/{sensor.get('name')}"
            for name, parent, child, element in [('lidar_mount_tf','base_link','wheelchair_lidar_mount',link),
                                                ('lidar_sensor_tf','wheelchair_lidar_mount',frame,sensor)]:
                p = element.find('pose')
                if p is not None and (p.get('relative_to') or p.get('rotation_format', 'euler_rpy') != 'euler_rpy' or p.get('degrees', 'false') == 'true'):
                    raise ValueError('LiDAR TF expects parent-relative poses in radians')
                x,y,z,roll,pitch,yaw = element.findtext('pose','0 0 0 0 0 0').split()
                actions.append(Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                    arguments=['--x',x,'--y',y,'--z',z,'--roll',roll,'--pitch',pitch,'--yaw',yaw,
                               '--frame-id',parent,'--child-frame-id',child], parameters=[{'use_sim_time':True}]))
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='wheelchair_bridge',
                        arguments=arguments, remappings=remappings, parameters=[{'use_sim_time':True}], output='screen'))
    extra = spec['options'].get('scenario_launch')
    if extra:
        scenario = scenario_share / extra
        if not scenario.is_file():
            raise ValueError(f'Scenario launch not found: {scenario}')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(scenario)),
            launch_arguments={'moving_traffic':value('moving_traffic'), 'world_name':spec['name']}.items()))
    return actions


def generate_launch_description():
    defaults = {'world':'wheelchair_world.sdf', 'world_package':'', 'headless':'false',
                'software_rendering':'false', 'moving_traffic':'true',
                'spawn_x':'', 'spawn_y':'', 'spawn_z':'', 'spawn_yaw':''}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] + [OpaqueFunction(function=setup)])
