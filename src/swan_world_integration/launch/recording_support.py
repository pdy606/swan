"""Read the recording launch contract without importing ROS or changing assets."""
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET


START_ZONES = ('apartment', 'market', 'city')


def boolean(value):
    value = str(value).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'Expected true or false, got {value!r}')
    return value == 'true'


def start_pose(path, zone):
    if zone not in START_ZONES:
        raise ValueError(f'start_zone must be one of {", ".join(START_ZONES)}')
    starts = json.loads(Path(path).read_text(encoding='utf-8'))['starts']
    pose = {key: float(starts[zone][key]) for key in ('x', 'y', 'z', 'yaw')}
    if not all(math.isfinite(value) for value in pose.values()):
        raise ValueError(f'Non-finite spawn pose for {zone}')
    return pose


def resource_path(models, inherited=''):
    """Prefer the preserved model even when another workspace has wheelchair."""
    paths = [str(Path(models).resolve())]
    paths.extend(part for part in inherited.split(os.pathsep) if part)
    return os.pathsep.join(dict.fromkeys(paths))


def world_name(path):
    root = ET.parse(path).getroot()
    worlds = root.findall('world')
    if root.tag != 'sdf' or len(worlds) != 1 or worlds[0].get('name') != 'swan_town':
        raise ValueError('The recording scene must contain exactly one world named swan_town')
    world = worlds[0]
    # This launch owns robot creation. An include can otherwise create a second robot.
    for model in world.iter('model'):
        if model.get('name') == 'wheelchair':
            raise ValueError('The recording world must not embed a wheelchair')
    for included in world.iter('include'):
        if (included.findtext('name') == 'wheelchair'
                or 'wheelchair' in included.findtext('uri', '')):
            raise ValueError('The recording world must not include a wheelchair')
    return world.get('name')


def local_pose(element):
    pose = element.find('pose')
    if pose is not None and (
            pose.get('relative_to')
            or pose.get('degrees', 'false').lower() == 'true'
            or pose.get('rotation_format', 'euler_rpy') != 'euler_rpy'):
        raise ValueError('Sensor TF requires parent-relative Euler poses in radians')
    values = tuple(float(x) for x in element.findtext('pose', '0 0 0 0 0 0').split())
    if len(values) != 6 or not all(math.isfinite(x) for x in values):
        raise ValueError('Invalid sensor pose')
    return values


def robot_contract(path):
    robot = ET.parse(path).getroot().find('model')
    if robot is None or robot.get('name') != 'wheelchair':
        raise ValueError('The recording model must be named wheelchair')
    if any(local_pose(robot)):
        raise ValueError('The preserved robot must have a zero model-relative pose')

    arguments = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    remappings = []
    transforms = []
    required = {
        'gz::sim::systems::JointController': '/model/wheelchair/front_drive/cmd_vel',
        'gz::sim::systems::JointPositionController': '/model/wheelchair/front_steering/cmd_pos',
    }
    for plugin_name, expected in required.items():
        plugin = robot.find(f"plugin[@name='{plugin_name}']")
        if plugin is None or plugin.findtext('topic') != expected:
            raise ValueError(f'Missing original steering/drive plugin: {plugin_name}')
        # One-way ROS -> Gazebo. Twist remains inside ROS and feeds the original controller.
        arguments.append(f'{expected}@std_msgs/msg/Float64]gz.msgs.Double')

    odometry = robot.find("plugin[@name='gz::sim::systems::OdometryPublisher']")
    if odometry is None:
        raise ValueError('Missing original odometry plugin')
    for key, ros_type, gz_type, destination in (
            ('odom_topic', 'nav_msgs/msg/Odometry', 'Odometry', '/odom'),
            ('tf_topic', 'tf2_msgs/msg/TFMessage', 'Pose_V', '/tf')):
        topic = odometry.findtext(key)
        if not topic:
            raise ValueError(f'Missing {key}')
        arguments.append(f'{topic}@{ros_type}[gz.msgs.{gz_type}')
        remappings.append((topic, destination))

    for kind, ros_type, gz_type, destination in (
            ('camera', 'Image', 'Image', '/camera'),
            ('gpu_lidar', 'LaserScan', 'LaserScan', '/scan')):
        sensors = [(link, sensor) for link in robot.findall('link')
                   for sensor in link.findall('sensor') if sensor.get('type') == kind]
        if len(sensors) != 1:
            raise ValueError(f'Expected one original {kind} sensor')
        link, sensor = sensors[0]
        mount = link.get('name')
        joints = [joint for joint in robot.findall('joint')
                  if joint.findtext('child') == mount]
        if (len(joints) != 1 or joints[0].get('type') != 'fixed'
                or joints[0].findtext('parent') != 'base_link'):
            raise ValueError(f'{mount} must be fixed to base_link for static sensor TF')
        topic = sensor.findtext('topic')
        if not topic:
            raise ValueError(f'Missing {kind} topic')
        arguments.append(f'{topic}@sensor_msgs/msg/{ros_type}[gz.msgs.{gz_type}')
        remappings.append((topic, destination))
        mount_frame = f'wheelchair/{mount}'
        sensor_frame = sensor.findtext('gz_frame_id') or f'{mount_frame}/{sensor.get("name")}'
        transforms.extend([
            (f'{mount}_mount_tf', 'base_link', mount_frame, local_pose(link)),
            (f'{mount}_sensor_tf', mount_frame, sensor_frame, local_pose(sensor)),
        ])
    return {'arguments': arguments, 'remappings': remappings, 'transforms': transforms}
