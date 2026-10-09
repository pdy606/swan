"""Offline checks for selecting the intended robot and bridging its real sensors."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('recording_support', PACKAGE / 'launch/recording_support.py')
support = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(support)


class RecordingContractTest(unittest.TestCase):
    def test_original_steering_model_does_not_receive_gazebo_twist(self):
        contract = support.robot_contract(PACKAGE / 'models/wheelchair/model.sdf')
        self.assertIn('/model/wheelchair/front_drive/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                      contract['arguments'])
        self.assertIn('/model/wheelchair/front_steering/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                      contract['arguments'])
        self.assertFalse(any('Twist' in arg for arg in contract['arguments']))

    def test_sensor_offsets_are_preserved_in_two_tf_edges(self):
        contract = support.robot_contract(PACKAGE / 'models/wheelchair/model.sdf')
        transforms = {child: (parent, pose) for _, parent, child, pose in contract['transforms']}
        mount_parent, mount = transforms['wheelchair/lidar_link']
        sensor_parent, sensor = transforms['wheelchair/lidar_link/lidar_sensor']
        self.assertEqual(mount_parent, 'base_link')
        self.assertEqual(sensor_parent, 'wheelchair/lidar_link')
        self.assertAlmostEqual(mount[0], 1.07)
        self.assertAlmostEqual(mount[2] + sensor[2], .13)
        self.assertAlmostEqual(transforms['wheelchair/camera_link'][1][4], .17)

    def test_preserved_model_has_priority_over_inherited_workspaces(self):
        own = PACKAGE / 'models'
        path = support.resource_path(own, '/other/models:' + str(own))
        self.assertEqual(path.split(':'), [str(own.resolve()), '/other/models'])

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

    def test_world_cannot_spawn_a_second_wheelchair(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'world.sdf'
            path.write_text('<sdf><world name="swan_town"/></sdf>', encoding='utf-8')
            self.assertEqual(support.world_name(path), 'swan_town')
            for content in ('<model name="wheelchair"/>',
                            '<include><uri>model://wheelchair</uri></include>'):
                path.write_text(f'<sdf><world name="swan_town">{content}</world></sdf>', encoding='utf-8')
                with self.assertRaises(ValueError):
                    support.world_name(path)


if __name__ == '__main__':
    unittest.main()
