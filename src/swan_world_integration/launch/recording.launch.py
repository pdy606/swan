"""Launch the integrated scene with the preserved team wheelchair for recording."""
import importlib.util
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_spec = importlib.util.spec_from_file_location(
    'swan_recording_support', Path(__file__).with_name('recording_support.py'))
_support = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_support)


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('swan_world_integration'))
    scene = share / 'worlds/swan_town.sdf'
    model = share / 'models/wheelchair/model.sdf'
    world = _support.world_name(scene)
    pose = _support.start_pose(share / 'config/integration.json', value('start_zone'))
    contract = _support.robot_contract(model)
    software = _support.boolean(value('software_rendering'))
    headless = _support.boolean(value('headless'))
    show_camera = _support.boolean(value('show_camera'))
    moving_traffic = _support.boolean(value('moving_traffic'))

    actions = [SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        _support.resource_path(share / 'models', os.environ.get('GZ_SIM_RESOURCE_PATH', '')))]
    if software:
        # This Mesa setup is also used by the existing UTM market launcher.
        actions.extend([
            SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
            UnsetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE'),
            SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        ])
    command = ['gz', 'sim', '-r']
    if headless:
        command.extend(['-s', '--headless-rendering'])
    actions.append(ExecuteProcess(cmd=command + [str(scene)], output='screen'))

    actions.append(Node(
        package='ros_gz_sim', executable='create', name='recording_wheelchair_spawn',
        arguments=['-world', world, '-file', str(model), '-name', 'wheelchair',
                   '-x', str(pose['x']), '-y', str(pose['y']), '-z', str(pose['z']),
                   '-Y', str(pose['yaw'])], output='screen'))
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='recording_wheelchair_bridge', arguments=contract['arguments'],
        remappings=contract['remappings'], parameters=[{'use_sim_time': True}], output='screen'))
    actions.append(Node(
        package='swan_world_integration', executable='swan_drive_controller.py',
        name='swan_drive_controller', parameters=[{'use_sim_time': True}], output='screen'))
    for name, parent, child, position in contract['transforms']:
        x, y, z, roll, pitch, yaw = map(str, position)
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name=name,
            arguments=['--x', x, '--y', y, '--z', z, '--roll', roll, '--pitch', pitch,
                       '--yaw', yaw, '--frame-id', parent, '--child-frame-id', child],
            parameters=[{'use_sim_time': True}], output='screen'))
    if moving_traffic:
        # This original script uses argparse and Gazebo Transport, not rclpy:
        # ExecuteProcess avoids injecting Node's unsupported --ros-args.
        animator = Path(get_package_prefix('swan_world_integration')) / 'lib/swan_world_integration/animate_crosswalk.py'
        actions.append(ExecuteProcess(
            cmd=[str(animator), '--world', world,
                 '--layout', str(share / 'config/traffic_layout.json'),
                 '--status', str(Path(tempfile.gettempdir()) / 'swan-town-traffic-status.json')],
            output='screen'))
    if show_camera:
        actions.append(Node(
            package='image_tools', executable='showimage', name='recording_camera',
            remappings=[('image', '/camera')], parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_zone', default_value='apartment',
                              choices=list(_support.START_ZONES),
                              description='Spawn once at the selected recording segment.'),
        DeclareLaunchArgument('software_rendering', default_value='false', choices=['true', 'false'],
                              description='Use Mesa llvmpipe on UTM when hardware rendering is unavailable.'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='Run Gazebo without its GUI; show_camera is a separate option.'),
        DeclareLaunchArgument('show_camera', default_value='true', choices=['true', 'false'],
                              description='Show the original wheelchair camera in an image_tools window.'),
        DeclareLaunchArgument('moving_traffic', default_value='true', choices=['true', 'false'],
                              description='Run the preserved market traffic and signal animation.'),
        OpaqueFunction(function=setup),
    ])
