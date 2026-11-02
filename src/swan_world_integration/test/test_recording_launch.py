"""Offline checks for selecting the intended robot and bridging its real sensors."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
SCENE = PACKAGE / 'worlds/swan_town.sdf'
SPEC = importlib.util.spec_from_file_location('recording_support', PACKAGE / 'launch/recording_support.py')
support = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(support)


class RecordingContractTest(unittest.TestCase):
    def test_original_steering_model_does_not_receive_gazebo_twist(self):
        contract = support.robot_contract(SCENE)
        self.assertIn('/model/wheelchair/front_drive/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                      contract['arguments'])
        self.assertIn('/model/wheelchair/front_steering/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                      contract['arguments'])
        self.assertFalse(any('Twist' in arg for arg in contract['arguments']))

    def test_sensor_offsets_are_preserved_in_two_tf_edges(self):
        contract = support.robot_contract(SCENE)
        transforms = {child: (parent, pose) for _, parent, child, pose in contract['transforms']}
        mount_parent, mount = transforms['wheelchair/lidar_link']
        sensor_parent, sensor = transforms['wheelchair/lidar_link/lidar_sensor']
        self.assertEqual(mount_parent, 'base_link')
        self.assertEqual(sensor_parent, 'wheelchair/lidar_link')
        self.assertAlmostEqual(mount[0], 1.07)
        self.assertAlmostEqual(mount[2] + sensor[2], .13)
        self.assertAlmostEqual(transforms['wheelchair/camera_link'][1][4], .17)

    def test_saved_world_is_standalone_with_one_embedded_robot(self):
        self.assertEqual(set(PACKAGE.rglob('*.sdf')), {SCENE})
        root, world, robot = support.read_world(SCENE)
        self.assertEqual(world.get('name'), 'swan_town')
        self.assertEqual(robot.get('name'), 'wheelchair')
        self.assertEqual(len(world.findall("model[@name='wheelchair']")), 1)
        for tag in ('include', 'mesh', 'uri'):
            self.assertIsNone(root.find(f'.//{tag}'))

    def test_spawn_uses_selected_segment_and_rejects_invalid_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'integration.json'
            document = {'starts': {
                'apartment': {'x': 9.8, 'y': 0, 'z': .87, 'yaw': math.pi},
                'market': {'x': -6.2, 'y': 0, 'z': .3, 'yaw': 0},
            }}
            path.write_text(json.dumps(document), encoding='utf-8')
            self.assertAlmostEqual(support.start_pose(path, 'apartment')['yaw'], math.pi)
            self.assertAlmostEqual(support.start_pose(path, 'market')['x'], -6.2)
            with self.assertRaises(ValueError):
                support.start_pose(path, '../../unexpected')
            document['starts']['market']['x'] = float('nan')
            path.write_text(json.dumps(document), encoding='utf-8')
            with self.assertRaises(ValueError):
                support.start_pose(path, 'market')

    def test_world_rejects_missing_duplicate_nested_or_external_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'world.sdf'
            robot = '<model name="wheelchair"><pose>1 2 3 0 0 1</pose></model>'
            path.write_text(f'<sdf><world name="swan_town">{robot}</world></sdf>', encoding='utf-8')
            self.assertEqual(support.world_name(path), 'swan_town')
            for content in ('', robot + robot,
                            f'<model name="wrapper">{robot}</model>',
                            robot + '<include><uri>model://wheelchair</uri></include>',
                            robot + '<model name="other"><link><visual><geometry><mesh/></geometry></visual></link></model>',
                            robot + '<uri>file://extra-material</uri>'):
                path.write_text(f'<sdf><world name="swan_town">{content}</world></sdf>', encoding='utf-8')
                with self.assertRaises(ValueError):
                    support.world_name(path)

    def test_apartment_uses_original_file_and_default_pose(self):
        before = SCENE.read_bytes()
        scene, temporary = support.prepare_scene(SCENE, PACKAGE / 'unused-settings.json', 'apartment')
        self.assertEqual(scene, SCENE)
        self.assertIsNone(temporary)
        self.assertEqual(SCENE.read_bytes(), before)

    def test_other_starts_only_change_robot_pose_and_clean_up(self):
        def without_robot_pose(root):
            robot = root.find("world/model[@name='wheelchair']")
            pose = robot.find('pose')
            if pose is not None:
                robot.remove(pose)
            return ET.canonicalize(ET.tostring(root, encoding='unicode'), strip_text=True)

        before = SCENE.read_bytes()
        unchanged = without_robot_pose(ET.fromstring(before))
        contract = support.robot_contract(SCENE)
        settings = PACKAGE / 'config/integration.json'
        for zone in ('market', 'city'):
            with self.subTest(zone=zone):
                scene, temporary = support.prepare_scene(SCENE, settings, zone)
                try:
                    self.assertNotEqual(scene, SCENE)
                    self.assertTrue(scene.is_file())
                    root, _, robot = support.read_world(scene)
                    pose = support.local_pose(robot)
                    expected = support.start_pose(settings, zone)
                    for index, key in ((0, 'x'), (1, 'y'), (2, 'z'), (5, 'yaw')):
                        self.assertAlmostEqual(pose[index], expected[key])
                    self.assertEqual(without_robot_pose(root), unchanged)
                    self.assertEqual(support.robot_contract(scene), contract)
                    self.assertEqual(SCENE.read_bytes(), before)
                finally:
                    temporary.cleanup()
                self.assertFalse(scene.exists())
                self.assertFalse(scene.parent.exists())


if __name__ == '__main__':
    unittest.main()
