"""World selection and spawn policy; deliberately independent of ROS imports."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def resolve_world(selection, package, share_lookup):
    robot_share = Path(share_lookup('wheelchair_gazebo'))
    aliases = json.loads((Path(share_lookup('swan_market')) / 'config/worlds.json').read_text())
    if not package and selection in aliases:
        entry = aliases[selection]
        package, selection = entry['package'], entry['world']
    share = Path(share_lookup(package or 'wheelchair_gazebo'))
    path = Path(selection).expanduser()
    if not path.is_absolute():
        path = share / 'worlds' / path
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f'World file not found: {path}')
    root = ET.parse(path).getroot()
    worlds = root.findall('world')
    if root.tag != 'sdf' or len(worlds) != 1 or not worlds[0].get('name'):
        raise ValueError(f'Expected one named SDF world: {path}')
    world = worlds[0]
    sidecar = path.with_suffix('.launch.json')
    options = json.loads(sidecar.read_text()) if sidecar.exists() else {}
    embedded = list(world.findall("model[@name='wheelchair']"))
    for inc in world.findall('include'):
        uri = inc.findtext('uri', '').rstrip('/')
        if inc.findtext('name') == 'wheelchair' or (inc.find('name') is None and uri == 'model://wheelchair'):
            if uri != 'model://wheelchair':
                raise ValueError('The common launcher expects model://wheelchair for an included wheelchair')
            embedded.append(inc)
    if len(embedded) > 1:
        raise ValueError('World contains more than one wheelchair')
    robot = embedded[0] if embedded and embedded[0].tag == 'model' else ET.parse(robot_share / 'models/wheelchair/model.sdf').getroot().find('model')
    return {'path':path, 'share':share, 'robot_share':robot_share,
            'name':world.get('name'), 'options':options,
            'embedded':bool(embedded), 'robot':robot}


def spawn_pose(spec, overrides):
    if spec['embedded']:
        if any(value != '' for value in overrides.values()):
            raise ValueError('This world already includes a wheelchair; edit its include/model pose instead of spawn_* arguments')
        return None
    pose = {'x':0., 'y':0., 'z':0.3, 'yaw':0.}
    pose.update(spec['options'].get('spawn', {}))
    for key in pose:
        if overrides.get(key, '') != '':
            pose[key] = float(overrides[key])
        pose[key] = float(pose[key])
        if not math.isfinite(pose[key]):
            raise ValueError(f'Spawn {key} must be finite')
    return pose
