#!/usr/bin/env python3
"""Assemble pinned environment copies; change only each model's world pose."""
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
SOURCES = [
    ('apartment', 'a23e859', 'src/wheelchair_gazebo/worlds/dy_scene2.sdf', [-35.98, 0, -0.05, math.pi]),
    ('city', '7adea9e', 'src/wheelchair_gazebo/worlds/wheelchair_world.sdf', [-14, 30, -0.025, 0]),
    ('market', '011fa32', 'src/wheelchair_gazebo/worlds/market_shopping.sdf', [0, 0, 0, 0]),
]


def read_git(ref, path):
    return subprocess.check_output(['git', 'show', f'{ref}:{path}'], cwd=REPO)


def internal_hash(model):
    element = copy.deepcopy(model)
    pose = element.find('pose')
    if pose is not None:
        element.remove(pose)
    value = ET.canonicalize(ET.tostring(element, encoding='unicode'), strip_text=True)
    return hashlib.sha256(value.encode()).hexdigest()


def transform_pose(pose, transform):
    x, y, z, roll, pitch, yaw = pose
    dx, dy, dz, theta = transform
    return [dx + math.cos(theta)*x - math.sin(theta)*y,
            dy + math.sin(theta)*x + math.cos(theta)*y,
            dz + z, roll, pitch, math.atan2(math.sin(yaw+theta), math.cos(yaw+theta))]


def box(world, name, pose, size, color):
    model = ET.SubElement(world, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    link = ET.SubElement(model, 'link', name='body')
    for kind in ['collision', 'visual']:
        element = ET.SubElement(link, kind, name=name+'_'+kind)
        geom = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
        ET.SubElement(geom, 'size').text = ' '.join(map(str, size))
        if kind == 'visual':
            mat = ET.SubElement(element, 'material')
            for field in ['ambient', 'diffuse']:
                ET.SubElement(mat, field).text = color
    return model


def main():
    root = ET.Element('sdf', version='1.10')
    world = ET.SubElement(root, 'world', name='swan_town')
    market = ET.fromstring(read_git('011fa32', SOURCES[2][2])).find('world')
    for item in market:
        if item.tag in ['physics', 'plugin', 'scene', 'light']:
            world.append(copy.deepcopy(item))
    manifest = {'world': 'swan_town', 'sources': [], 'starts': {
        'apartment': {'x': -45.78, 'y': 0, 'z': .82, 'yaw': 0},
        'market': {'x': -6.2, 'y': 0, 'z': .30, 'yaw': 0},
        'city': {'x': -18.5, 'y': 15.6, 'z': .375, 'yaw': math.pi/2},
    }, 'connectors': []}
    names = set()
    for zone, ref, path, transform in SOURCES:
        commit = subprocess.check_output(['git', 'rev-parse', ref], cwd=REPO, text=True).strip()
        data = read_git(commit, path)
        source = ET.fromstring(data).find('world')
        record = {'zone': zone, 'commit': commit, 'path': path,
                  'sha256': hashlib.sha256(data).hexdigest(), 'transform_xyz_yaw': transform,
                  'models': {}, 'source_robot_includes': [ET.tostring(i, encoding='unicode') for i in source.findall('include')]}
        for original in source.findall('model'):
            name = original.get('name')
            if name in names:
                raise ValueError(f'Duplicate model name: {name}')
            names.add(name)
            model = copy.deepcopy(original)
            values = list(map(float, original.findtext('pose', '0 0 0 0 0 0').split()))
            pose = model.find('pose')
            if pose is None:
                pose = ET.Element('pose')
                model.insert(0, pose)
            if pose.attrib:
                raise ValueError(f'Unexpected pose attributes in {zone}/{name}: {pose.attrib}')
            pose.text = ' '.join(f'{v:.12g}' for v in transform_pose(values, transform))
            assert internal_hash(original) == internal_hash(model)
            world.append(model)
            record['models'][name] = {'internal_sha256': internal_hash(original), 'original_pose': values}
        manifest['sources'].append(record)
    # New geometry lives outside imported footprints. Original grounds remain intact.
    connectors = [
        ('integration_parking_market_path', [-14, 0, -.10, 0, 0, 0], [12, 3.4, .20]),
        ('integration_city_branch', [-18.5, 7.60, -.10, 0, 0, 0], [2, 11.80, .20]),
    ]
    for name, pose, size in connectors:
        box(world, name, pose, size, '.62 .62 .61 1')
        manifest['connectors'].append(name)
    # New access ramp rises from Z=0 to the city's unchanged sidewalk top Z=.075.
    rise, run, thickness = .075, 1.5, .10
    angle = math.atan2(rise, run)
    box(world, 'integration_city_access_ramp',
        [-18.5, 14.25 + thickness/2*math.sin(angle), rise/2-thickness/2*math.cos(angle), angle, 0, 0],
        [2, run/math.cos(angle), thickness], '.62 .62 .61 1')
    manifest['connectors'].append('integration_city_access_ramp')
    manifest['notes'] = [
        'All source environment models are retained. Only their top-level world pose is transformed.',
        'World-level physics, lighting and renderer are shared from the market world; source worlds are unmodified.',
        'Source robot includes are not duplicated. The dedicated launcher spawns one unchanged dayeong robot.',
        'The city ground is at Z=-.025, apartment ground at Z=-.05; imported grounds and dimensions are preserved.',
        'The latest apartment source has an open fixed door frame, not animated door panels.',
    ]
    ET.indent(root, space='  ')
    (PACKAGE/'worlds/swan_town.sdf').write_bytes(ET.tostring(root, encoding='utf-8', xml_declaration=True)+b'\n')
    (PACKAGE/'config/integration.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    traffic = json.loads(read_git('011fa32', 'src/swan_market/config/market_layout.json'))
    traffic['world'] = 'swan_town'
    (PACKAGE/'config/traffic_layout.json').write_text(json.dumps(traffic, ensure_ascii=False, indent=2)+'\n')
    print(f'Integrated {len(names)} unchanged environment models + {len(manifest["connectors"])} new connectors.')


if __name__ == '__main__':
    main()
