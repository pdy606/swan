"""Generate experiment assets ONLY inside this package; never edit SWAN sources."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(),
            tuple(canonical(child) for child in element))


def generate(package_root, swan_src=None, test_spawn=None, test_wall=False, test_obstacle_width=8.0):
    root = Path(package_root).resolve()
    src = Path(swan_src).resolve() if swan_src else root.parent
    model_source = src/'wheelchair_gazebo/models/wheelchair/model.sdf'
    world_source = src/'wheelchair_gazebo/worlds/wheelchair_world.sdf'
    model_original = ET.parse(model_source).getroot()
    world_original = ET.parse(world_source).getroot()
    model = ET.fromstring(ET.tostring(model_original))
    base = model.find("./model/link[@name='base_link']")
    if base is None:
        raise ValueError('Original base_link missing')
    sensor = ET.fromstring('''<sensor name="camera_driver_sensor" type="camera">
      <pose>0 0 1.2 0 0.1308996938995747 0</pose>
      <always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>
      <topic>/camera_driver</topic><camera>
        <horizontal_fov>1.5707963267948966</horizontal_fov>
        <image><width>320</width><height>240</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>100</far></clip>
      </camera></sensor>''')
    base.append(sensor)
    output = root/'runtime/generated'
    model_dir = output/'wheelchair_driver'
    model_dir.mkdir(parents=True, exist_ok=True)
    model_path = model_dir/'model.sdf'
    ET.ElementTree(model).write(model_path, encoding='utf-8', xml_declaration=True)
    (model_dir/'model.config').write_text('''<?xml version="1.0"?>
<model><name>wheelchair_driver</name><version>1.0</version>
<sdf version="1.9">model.sdf</sdf><author><name>SWAN</name></author>
<description>Original wheelchair with one massless driver RGB sensor.</description></model>
''')
    world = ET.fromstring(ET.tostring(world_original))
    includes = [item for item in world.findall('./world/include')
                if item.findtext('uri') == 'model://wheelchair']
    if len(includes) != 1:
        raise ValueError('Expected exactly one original wheelchair include')
    includes[0].find('uri').text = model_dir.as_uri()
    original_spawn = includes[0].findtext('pose')
    if test_spawn is not None:
        if len(test_spawn) != 6 or not all(math.isfinite(v) for v in test_spawn):
            raise ValueError('Test spawn must be six finite numbers')
        includes[0].find('pose').text = ' '.join(str(v) for v in test_spawn)
    if test_wall:
        if test_spawn is None or test_spawn[3:] != [0, 0, 0]:
            raise ValueError('Wall fixture requires an upright, forward-facing test spawn')
        if not math.isfinite(test_obstacle_width) or test_obstacle_width <= 0:
            raise ValueError('Test obstacle width must be positive')
        # Front laser x=.932; near face x=1.332 gives 0.40m raw clearance.
        fixture = ET.fromstring(f'''<model name="ai_test_wall"><static>true</static>
          <pose>{test_spawn[0]+1.432} {test_spawn[1]} 0.75 0 0 0</pose><link name="wall">
          <collision name="wall"><geometry><box><size>0.20 {test_obstacle_width} 1.5</size></box></geometry></collision>
          <visual name="wall"><geometry><box><size>0.20 {test_obstacle_width} 1.5</size></box></geometry>
          <material><ambient>0.7 0.2 0.1 1</ambient><diffuse>0.7 0.2 0.1 1</diffuse></material></visual>
          </link></model>''')
        world.find('world').append(fixture)
    world_path = output/'wheelchair_driver_world.sdf'
    ET.ElementTree(world).write(world_path, encoding='utf-8', xml_declaration=True)
    # Proof: all original SDF semantics survive after reversing the ONLY two additions.
    check_model = ET.parse(model_path).getroot()
    check_base = check_model.find("./model/link[@name='base_link']")
    check_base.remove(check_base.find("sensor[@name='camera_driver_sensor']"))
    check_world = ET.parse(world_path).getroot()
    check_include = next(item for item in check_world.findall('./world/include')
                         if item.findtext('uri') == model_dir.as_uri())
    check_include.find('uri').text = 'model://wheelchair'
    if test_spawn is not None:
        check_include.find('pose').text = original_spawn
    if test_wall:
        check_world.find('world').remove(check_world.find("./world/model[@name='ai_test_wall']"))
    if canonical(check_model) != canonical(model_original) or canonical(check_world) != canonical(world_original):
        raise ValueError('Derived assets changed original semantics')
    manifest = {'model_source': str(model_source), 'model_source_sha256': digest(model_source),
                'world_source': str(world_source), 'world_source_sha256': digest(world_source),
                'model': str(model_path), 'world': str(world_path),
                'preservation_verified': True, 'test_spawn': test_spawn, 'test_wall': test_wall,
                'test_obstacle_width': test_obstacle_width,
                'camera_pose_base': [0, 0, 1.2, 0, 0.1308996938995747, 0]}
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return world_path
