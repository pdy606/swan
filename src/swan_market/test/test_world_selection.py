"""Offline configuration and launch-action tests; not Gazebo runtime tests."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent
GAZEBO = SRC / 'wheelchair_gazebo'


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


support = load(ROOT / 'launch/world_support.py', 'support')


def share(name):
    path = SRC / name
    if not path.is_dir():
        raise LookupError(name)
    return str(path)


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Config:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def launch_module():
    # Inspect the generated graph without requiring ROS on the developer's Mac.
    modules = {}
    for name in ['ament_index_python', 'ament_index_python.packages', 'launch',
                 'launch.actions', 'launch.launch_description_sources',
                 'launch.substitutions', 'launch_ros', 'launch_ros.actions']:
        modules[name] = types.ModuleType(name)
    modules['ament_index_python.packages'].get_package_share_directory = share
    modules['launch'].LaunchDescription = Action
    for name in ['DeclareLaunchArgument','ExecuteProcess','IncludeLaunchDescription',
                 'OpaqueFunction','SetEnvironmentVariable','UnsetEnvironmentVariable']:
        setattr(modules['launch.actions'], name, type(name, (Action,), {}))
    modules['launch.launch_description_sources'].PythonLaunchDescriptionSource = Action
    modules['launch.substitutions'].LaunchConfiguration = Config
    modules['launch_ros.actions'].Node = type('Node', (Action,), {})
    with patch.dict(sys.modules, modules):
        return load(ROOT / 'launch/world.launch.py', 'simulation_test')


class WorldSelectionTest(unittest.TestCase):
    def setUp(self):
        self.launch = launch_module()
        self.context = dict(world='wheelchair_world.sdf', world_package='', headless='false',
                            software_rendering='false', moving_traffic='true',
                            spawn_x='', spawn_y='', spawn_z='', spawn_yaw='')

    def actions(self, **kwargs):
        return self.launch.setup({**self.context, **kwargs})

    def test_embedded_robot_is_not_spawned_twice(self):
        actions = self.actions()
        self.assertFalse(any(a.kwargs.get('executable') == 'create' for a in actions))
        self.assertTrue(any('wheelchair_world.sdf' in str(a.kwargs.get('cmd')) for a in actions))

    def test_nine_team_worlds_are_selectable_with_one_spawn(self):
        files = sorted(p for p in (GAZEBO / 'worlds').glob('*.world') if p.name != 'market_shopping.world')
        self.assertEqual(len(files), 9)
        for path in files:
            with self.subTest(world=path.name):
                spec = support.resolve_world(path.name, '', share)
                creates = [a for a in self.actions(world=path.name) if a.kwargs.get('executable') == 'create']
                self.assertEqual(len(creates), 1)
                self.assertEqual(creates[0].kwargs['arguments'][1], spec['name'])
                self.assertFalse(any(type(a).__name__ == 'IncludeLaunchDescription' for a in self.actions(world=path.name)))

    def test_market_spawn_and_scenario(self):
        actions = self.actions(world='market', moving_traffic='false')
        create = next(a for a in actions if a.kwargs.get('executable') == 'create')
        args = create.kwargs['arguments']
        self.assertEqual(args[1], 'swan_market')
        self.assertEqual(float(args[args.index('-x')+1]), -6.2)
        extra = next(a for a in actions if type(a).__name__ == 'IncludeLaunchDescription')
        self.assertEqual(dict(extra.kwargs['launch_arguments']), {'moving_traffic':'false', 'world_name':'swan_market'})
        self.assertIn('swan_market/models', str(actions[0].args))
        self.assertIn(str(SRC/'swan_market/launch/traffic.launch.py'), str(extra.args[0].args))

    def test_explicit_package_matches_alias(self):
        a = support.resolve_world('market', '', share)
        b = support.resolve_world('market_shopping.world', 'wheelchair_gazebo', share)
        self.assertEqual(a['path'], b['path'])
        self.assertEqual(a['path'], GAZEBO/'worlds/market_shopping.world')
        self.assertFalse((SRC/'swan_market/worlds/market_shopping.sdf').exists())

    def test_absolute_path_and_spawn_override(self):
        path = GAZEBO / 'worlds/layout_narrow_alley.world'
        spec = support.resolve_world(str(path), '', share)
        pose = support.spawn_pose(spec, {'x':'1.2', 'y':'-3', 'yaw':'1.57'})
        self.assertEqual(pose['x'], 1.2)
        self.assertEqual(pose['y'], -3)

    def test_existing_robot_rejects_ignored_spawn_override(self):
        with self.assertRaisesRegex(ValueError, 'already includes'):
            self.actions(spawn_x='2')

    def test_missing_file_and_model_only_file_fail_clearly(self):
        with self.assertRaisesRegex(ValueError, 'not found'):
            support.resolve_world('missing.world', '', share)
        with self.assertRaisesRegex(ValueError, 'one named SDF world'):
            support.resolve_world(str(GAZEBO/'models/wheelchair/model.sdf'), '', share)

    def test_nonfinite_spawn_is_rejected(self):
        spec = support.resolve_world('market', '', share)
        with self.assertRaisesRegex(ValueError, 'finite'):
            support.spawn_pose(spec, {'x':'nan'})

    def test_camera_and_scan_bridges_use_model_topics(self):
        bridge = next(a for a in self.actions(world='market') if a.kwargs.get('executable') == 'parameter_bridge')
        self.assertIn(('/camera', '/camera'), bridge.kwargs['remappings'])
        self.assertIn(('/model/wheelchair/scan', '/scan'), bridge.kwargs['remappings'])
        self.assertIn('/model/wheelchair/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist', bridge.kwargs['arguments'])

    def test_camera_optional_for_other_branch_model(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)/'test.world'
            p.write_text('<sdf version="1.9"><world name="other"><model name="wheelchair"><link name="base_link"/></model></world></sdf>')
            self.assertTrue(self.actions(world=str(p)))

    def test_headless_and_software_rendering(self):
        actions = self.actions(world='market', headless='true', software_rendering='true')
        cmd = next(a.kwargs['cmd'] for a in actions if 'cmd' in a.kwargs)
        self.assertIn('--headless-rendering', cmd)
        self.assertTrue(any(a.args == ('GALLIUM_DRIVER','llvmpipe') for a in actions))

    def test_robot_links_joints_and_sensors_have_unique_names(self):
        spec = support.resolve_world('market', '', share)
        robot = spec['robot']
        for tag in ('link', 'joint'):
            names = [item.get('name') for item in robot.findall(tag)]
            self.assertEqual(len(names), len(set(names)), f'duplicate {tag}')
        for link in robot.findall('link'):
            names = [sensor.get('name') for sensor in link.findall('sensor')]
            self.assertEqual(len(names), len(set(names)), 'duplicate sensor')

    def test_market_generator_metadata_is_consistent(self):
        spec = support.resolve_world('market', '', share)
        layout = json.loads((SRC/'swan_market/config/market_layout.json').read_text())
        self.assertEqual(spec['options']['spawn'], layout['spawn'])


traffic = load(ROOT/'scripts/animate_crosswalk.py', 'traffic')


class SignalCycleTest(unittest.TestCase):
    def setUp(self):
        self.layout=json.loads((ROOT/'config/market_layout.json').read_text())
        self.cfg=self.layout['traffic']

    def test_red_green_and_blink_boundaries(self):
        for time,state,red,green in [(0,'red',True,False),(16.999,'red',True,False),
              (17,'green',False,True),(23.999,'green',False,True),
              (24,'flashing_green',False,True),(24.5,'flashing_green',False,False),
              (25,'flashing_green',False,True),(26.5,'flashing_green',False,False),
              (27,'red',True,False),(32,'red',True,False),(49,'green',False,True)]:
            with self.subTest(time=time):
                self.assertEqual(traffic.signal_state(time,self.cfg),(state,red,green))

    def test_signals_do_not_show_green_during_motorcycle_motion(self):
        for i in range(1280):
            t=i*.05
            _,stage,_=traffic.traffic_poses(t,self.cfg)
            state,red,green=traffic.signal_state(t,self.cfg)
            self.assertFalse(red and green)
            if stage=='motorcycles':self.assertEqual(state,'red')
            if stage=='pedestrians':self.assertNotEqual(state,'red')
            lamps=traffic.signal_poses(t,self.cfg)
            for signal in self.cfg['signals']:
                self.assertEqual(lamps[signal['red']][3],0 if red else -5)
                self.assertEqual(lamps[signal['green']][3],0 if green else -5)

    def test_icons_are_collision_free_and_housed_on_both_banks(self):
        import xml.etree.ElementTree as ET
        world=ET.parse(GAZEBO/'worlds/market_shopping.world').getroot().find('world')
        self.assertEqual(len(self.cfg['signals']),2)
        for signal in self.cfg['signals']:
            for color in ('red','green'):
                icon=world.find(f"model[@name='{signal[color]}']")
                self.assertIsNotNone(icon)
                self.assertFalse(icon.findall('.//collision'))
                self.assertGreater(len(icon.findall('.//emissive')),5)
        ys=[p[1] for p in self.layout['aisle_centerline']]
        self.assertGreater(max(ys),2.1)
        self.assertLess(min(ys),-2.1)
        self.assertEqual(self.layout['route'][0],[-6.2,0])


if __name__ == '__main__':
    unittest.main()
