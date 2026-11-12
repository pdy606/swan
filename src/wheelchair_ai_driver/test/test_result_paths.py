"""Local environment datasets: preserve historical bytes, resume, never mix."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from wheelchair_ai_driver.result_paths import (
    active_results, activate_results, select_results, all_attempts, restore_csvs_if_needed,
)
from wheelchair_ai_driver.experiment_store import ExperimentStore

ROOT = Path(__file__).resolve().parents[1]


class LocalResultsTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=ROOT / 'runtime')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.a = {'schema': 2, 'contact_plugin_sha256': 'computer-A'}
        self.b = {'schema': 2, 'contact_plugin_sha256': 'computer-B'}

    def commit(self, directory, fingerprint, number=1):
        store = ExperimentStore(directory, fingerprint)
        for mode in ('off', 'on'):
            store.commit(mode, number, {'collisions': 1, 'progress_percent': 45., 'sim_seconds': 12.,
                                       'avoidance_interventions': 2, 'reason': 'test_failure'})
        return store

    def test_fresh_clone_starts_at_one_and_fingerprint_is_created_only_on_start(self):
        directory = select_results(self.root, self.a)
        self.assertEqual(directory, self.root / 'results')
        self.assertFalse(directory.exists())
        directory = select_results(self.root, self.a, create=True)
        self.assertEqual(json.loads((directory / 'experiment.json').read_text()), self.a)
        self.assertFalse((directory / 'records').exists())
        store = self.commit(directory, self.a)
        self.assertTrue(store.completed('off', 1))
        self.assertFalse(store.completed('off', 2))
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            store.commit('off', 1, {})

    def test_changed_environment_preserves_all_previous_data_and_returns_to_matching_dataset(self):
        original = select_results(self.root, self.a, create=True)
        self.commit(original, self.a)
        before = {p.relative_to(original): p.read_bytes() for p in original.rglob('*') if p.is_file()}
        other = select_results(self.root, self.b, create=True)
        self.assertEqual(other.parent, original / 'environments')
        activate_results(self.root, other)
        self.assertEqual(active_results(self.root), other)
        store = self.commit(other, self.b)
        self.assertFalse(store.completed('on', 2))
        self.assertEqual(select_results(self.root, self.a), original)
        activate_results(self.root, original)
        self.assertEqual(select_results(self.root, self.b), other)
        (self.root / 'runtime/active_results.json').unlink()
        self.assertEqual(select_results(self.root, self.b), other)
        for relative, data in before.items():
            self.assertEqual((original / relative).read_bytes(), data)

    def test_missing_or_corrupt_manifest_never_adopts_unknown_results(self):
        base = self.root / 'results'
        base.mkdir()
        csv = base / 'swan_off.csv'
        csv.write_text('private existing data')
        selected = select_results(self.root, self.a, create=True)
        self.assertNotEqual(selected, base)
        self.assertEqual(csv.read_text(), 'private existing data')
        (base / 'experiment.json').write_text('{corrupt')
        self.assertEqual(select_results(self.root, self.a), selected)
        self.assertEqual((base / 'experiment.json').read_text(), '{corrupt')

    def test_setup_preview_never_creates_or_rewrites_fingerprints(self):
        original = select_results(self.root, self.a, create=True)
        manifest = original / 'experiment.json'
        before = (manifest.read_bytes(), manifest.stat().st_mtime_ns)
        different = select_results(self.root, self.b)
        self.assertFalse(different.exists())
        self.assertEqual(before, (manifest.read_bytes(), manifest.stat().st_mtime_ns))

    def test_active_pointer_cannot_escape_local_results(self):
        runtime = self.root / 'runtime'
        runtime.mkdir()
        (runtime / 'active_results.json').write_text(json.dumps({'directory': '../outside'}))
        with self.assertRaises(ValueError):
            active_results(self.root)
        with self.assertRaises(ValueError):
            activate_results(self.root, self.root / 'outside')

    def test_cleanup_includes_previous_environment_attempts(self):
        original = select_results(self.root, self.a, create=True)
        other = select_results(self.root, self.b, create=True)
        for directory in (original, other):
            path = directory / 'runs/off_001_test/attempt.json'
            path.parent.mkdir(parents=True)
            path.write_text('{}')
        activate_results(self.root, other)
        self.assertEqual(len(all_attempts(self.root)), 2)

    def test_committed_results_restore_stale_csv_and_valid_csv_is_not_rewritten(self):
        directory = select_results(self.root, self.a, create=True)
        store = self.commit(directory, self.a)
        csv = directory / 'swan_on.csv'
        original = csv.read_bytes()
        before = csv.stat().st_mtime_ns
        restore_csvs_if_needed(directory, self.a)
        self.assertEqual(csv.stat().st_mtime_ns, before)
        store.commit('on', 2, {'collisions': 2, 'progress_percent': 50., 'sim_seconds': 14.,
                               'avoidance_interventions': 1, 'reason': 'test_failure'})
        correct = csv.read_bytes()
        csv.write_bytes(original)  # simulate a committed JSON with an older CSV
        restore_csvs_if_needed(directory, self.a)
        self.assertEqual(csv.read_bytes(), correct)
        csv.unlink()
        restore_csvs_if_needed(directory, self.a)
        self.assertEqual(csv.read_bytes(), correct)

    def test_verifier_uses_active_environment_without_legacy_preservation_snapshot(self):
        original = select_results(self.root, self.a, create=True)
        self.commit(original, self.a)
        before = {p.relative_to(original): p.read_bytes() for p in original.rglob('*') if p.is_file()}
        other = select_results(self.root, self.b, create=True)
        activate_results(self.root, other)
        store = ExperimentStore(other, self.b)
        result = {'collisions': 0, 'progress_percent': 30., 'sim_seconds': 10.,
                  'start_sim': 1., 'end_sim': 11., 'goal_reached': False, 'rtf': .5,
                  'avoidance_interventions': 0, 'reason': 'test_failure'}
        for mode in ('off', 'on'):
            directory = other / f'runs/{mode}_001_test'
            directory.mkdir(parents=True)
            (directory / 'result.json').write_text(json.dumps(result))
            for name in ('contacts.jsonl', 'validation.jsonl', 'launch.log'):
                (directory / name).write_text('')
            store.commit(mode, 1, result)
        tools = self.root / 'tools'
        tools.mkdir()
        script = tools / 'verify_experiments.py'
        shutil.copy2(ROOT / 'tools/verify_experiments.py', script)
        process = subprocess.run([sys.executable, '-B', str(script)], check=True, capture_output=True,
                                 text=True, env=dict(os.environ, PYTHONPATH=str(ROOT), PYTHONDONTWRITEBYTECODE='1'))
        report = json.loads(process.stdout)
        self.assertEqual(report['validated_results'], 2)
        self.assertEqual(report['original_and_frozen_files_checked'], 0)
        self.assertTrue((other / 'provenance/verification.json').is_file())
        for relative, data in before.items():
            self.assertEqual((original / relative).read_bytes(), data)


if __name__ == '__main__':
    unittest.main()
