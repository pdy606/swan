"""Audit source preservation without ROS, Gazebo, network access or regeneration.

Manifest checks always run. When pinned commits are available in a local Git
checkout, additional checks authenticate that manifest against the source bytes.
Set SWAN_SOURCE_REPO to use a different existing checkout; nothing is fetched.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


def canonical_xml(element):
    return ET.canonicalize(ET.tostring(element, encoding='unicode'), strip_text=True)


def internal_digest(model):
    """Exclude only the model's direct world pose, retaining all inner content."""
    inner = copy.deepcopy(model)
    for child in list(inner):
        if child.tag == 'pose':
            inner.remove(child)
    return hashlib.sha256(canonical_xml(inner).encode('utf-8')).hexdigest()


def pose_values(element):
    return [float(value) for value in element.findtext('pose', '0 0 0 0 0 0').split()]


def available_source_repo():
    override = os.environ.get('SWAN_SOURCE_REPO')
    if override:
        return Path(override).expanduser()
    return next((parent for parent in PACKAGE.parents if (parent / '.git').exists()), None)


class PreservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads((PACKAGE / 'config/integration.json').read_text())
        cls.vendors = json.loads((PACKAGE / 'config/vendor_sources.json').read_text())
        cls.root = ET.parse(PACKAGE / 'worlds/swan_town.sdf').getroot()
        cls.world = cls.root.find('world')
        cls.models = cls.world.findall('model')
        cls.by_name = {model.get('name'): model for model in cls.models}
        cls.repo = available_source_repo()

    def source_blob(self, commit, path):
        if self.repo is None or not self.repo.is_dir():
            self.skipTest('Local source Git checkout unavailable; manifest audits still run')
        try:
            result = subprocess.run(
                ['git', 'show', f'{commit}:{path}'], cwd=self.repo,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
            )
        except FileNotFoundError:
            self.skipTest('Git unavailable; manifest audits still run')
        if result.returncode:
            self.skipTest(f'Pinned Git source unavailable: {commit[:7]}:{path}')
        return result.stdout

    def test_world_directory_has_exactly_one_sdf(self):
        assets = {path.relative_to(PACKAGE / 'worlds').as_posix()
                  for path in (PACKAGE / 'worlds').rglob('*') if path.is_file()}
        self.assertEqual(assets, {'swan_town.sdf'})
        self.assertEqual(self.root.tag, 'sdf')
        self.assertEqual(len(self.root.findall('world')), 1)
        self.assertEqual(self.world.get('name'), self.manifest['world'])

    def test_source_models_and_only_declared_connectors_are_present(self):
        source_names = [name for source in self.manifest['sources'] for name in source['models']]
        self.assertEqual(len(source_names), len(set(source_names)), 'Source model name collision')
        self.assertEqual(len(self.models), len(self.by_name), 'Duplicate model in assembled world')
        connectors = self.manifest['connectors']
        self.assertEqual(len(connectors), len(set(connectors)))
        self.assertFalse(set(source_names) & set(connectors))
        self.assertEqual(set(self.by_name), set(source_names) | set(connectors))

    def test_model_internals_equal_preservation_manifest(self):
        for source in self.manifest['sources']:
            for name, original in source['models'].items():
                with self.subTest(zone=source['zone'], model=name):
                    self.assertEqual(internal_digest(self.by_name[name]), original['internal_sha256'])

    def test_top_level_poses_follow_one_rigid_transform_per_zone(self):
        for source in self.manifest['sources']:
            dx, dy, dz, rotation = source['transform_xyz_yaw']
            c, s = math.cos(rotation), math.sin(rotation)
            for name, record in source['models'].items():
                with self.subTest(zone=source['zone'], model=name):
                    model = self.by_name[name]
                    self.assertEqual(len(model.findall('pose')), 1)
                    self.assertFalse(model.find('pose').attrib, 'Unexpected pose frame or units')
                    actual = pose_values(model)
                    self.assertEqual(len(actual), 6)
                    self.assertTrue(all(math.isfinite(value) for value in actual))
                    x, y, z, roll, pitch, yaw = record['original_pose']
                    expected = [dx + c*x - s*y, dy + s*x + c*y, dz + z, roll, pitch]
                    for measured, desired in zip(actual[:5], expected):
                        self.assertAlmostEqual(measured, desired, delta=1e-8)
                    # Angles differing by 2*pi represent the same preserved orientation.
                    self.assertAlmostEqual(math.sin(actual[5]), math.sin(yaw + rotation), delta=1e-9)
                    self.assertAlmostEqual(math.cos(actual[5]), math.cos(yaw + rotation), delta=1e-9)

    def test_manifest_matches_pinned_git_sources_when_available(self):
        for source in self.manifest['sources']:
            with self.subTest(zone=source['zone']):
                data = self.source_blob(source['commit'], source['path'])
                self.assertEqual(hashlib.sha256(data).hexdigest(), source['sha256'])
                original_world = ET.fromstring(data).find('world')
                original_models = original_world.findall('model')
                originals = {model.get('name'): model for model in original_models}
                self.assertEqual(len(original_models), len(originals))
                self.assertEqual(set(originals), set(source['models']))
                for name, model in originals.items():
                    self.assertEqual(internal_digest(model), source['models'][name]['internal_sha256'], name)
                    self.assertEqual(pose_values(model), source['models'][name]['original_pose'], name)
                recorded_includes = [ET.fromstring(text) for text in source['source_robot_includes']]
                self.assertEqual([canonical_xml(item) for item in original_world.findall('include')],
                                 [canonical_xml(item) for item in recorded_includes])

    def test_vendor_files_match_byte_hashes(self):
        targets = [item['target_path'] for item in self.vendors]
        self.assertEqual(len(targets), len(set(targets)))
        self.assertIn('models/wheelchair/model.sdf', targets)
        for item in self.vendors:
            with self.subTest(file=item['target_path']):
                copied = (PACKAGE / item['target_path']).read_bytes()
                self.assertEqual(hashlib.sha256(copied).hexdigest(), item['sha256'])

    def test_vendor_files_equal_pinned_git_bytes_when_available(self):
        for item in self.vendors:
            with self.subTest(file=item['target_path']):
                original = self.source_blob(item['commit'], item['source_path'])
                self.assertEqual((PACKAGE / item['target_path']).read_bytes(), original)

    def test_robot_is_separate_and_not_duplicated_in_world(self):
        self.assertFalse(list(self.world.iter('include')), 'Environment must not spawn included robots')
        self.assertFalse([model for model in self.world.iter('model')
                          if 'wheelchair' == model.get('name')])
        robot = ET.parse(PACKAGE / 'models/wheelchair/model.sdf').getroot()
        self.assertEqual(len(robot.findall('model')), 1)
        self.assertEqual(robot.find('model').get('name'), 'wheelchair')
        self.assertFalse(robot.findall('world'))

    def test_shared_world_settings_have_no_duplicate_systems(self):
        self.assertEqual(len(self.world.findall('physics')), 1)
        self.assertEqual(len(self.world.findall('scene')), 1)
        plugins = [plugin.get('name') for plugin in self.world.findall('plugin')]
        self.assertEqual(len(plugins), len(set(plugins)))
        self.assertEqual(set(plugins), {
            'gz::sim::systems::Physics', 'gz::sim::systems::UserCommands',
            'gz::sim::systems::SceneBroadcaster', 'gz::sim::systems::Sensors',
        })

    def test_shared_world_settings_equal_market_source_when_available(self):
        market = next(source for source in self.manifest['sources'] if source['zone'] == 'market')
        original = ET.fromstring(self.source_blob(market['commit'], market['path'])).find('world')
        fields = {'physics', 'plugin', 'scene', 'light'}
        self.assertEqual([canonical_xml(item) for item in self.world if item.tag in fields],
                         [canonical_xml(item) for item in original if item.tag in fields])


if __name__ == '__main__':
    unittest.main()
