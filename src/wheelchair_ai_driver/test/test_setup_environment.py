"""Provisioning checks use temporary files; no installed model/environment is edited."""
import hashlib
import fcntl
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('setup_environment', ROOT / 'tools/setup_environment.py')
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)


class SetupTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=ROOT / 'runtime')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.directory = self.root / 'assets/navigation_model_zoo/MBRA_PG_Official'
        self.directory.mkdir(parents=True)
        self.payload = b'fake model for downloader tests only'
        self.manifest = {'repository': 'UCLA-VAIL/Navigation-Model-Zoo-Public',
                         'revision': 'fixed-commit-for-test', 'files': {}}
        for name, content in [('inference.py', b'# reference'), ('model_info.yaml', b'name: MBRA'),
                              ('mbra.onnx', self.payload)]:
            self.manifest['files'][name] = {'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest()}
            if name != 'mbra.onnx':
                (self.directory / name).write_bytes(content)
        (self.directory / 'manifest.json').write_text(json.dumps(self.manifest))

    def test_download_pinned_url_verify_and_skip_second_network_request(self):
        seen = []
        def opener(request, timeout):
            seen.append(request.full_url)
            return io.BytesIO(self.payload)
        setup.ensure_model(self.root, opener=opener)
        model = self.directory / 'mbra.onnx'
        before = model.stat().st_mtime_ns
        setup.ensure_model(self.root, opener=opener)
        self.assertEqual(seen, ['https://huggingface.co/UCLA-VAIL/Navigation-Model-Zoo-Public/'
                                'resolve/fixed-commit-for-test/MBRA_PG_Official/mbra.onnx'])
        self.assertEqual(model.read_bytes(), self.payload)
        self.assertEqual(model.stat().st_mtime_ns, before)

    def test_existing_bad_model_is_never_overwritten(self):
        model = self.directory / 'mbra.onnx'
        model.write_bytes(b'keep existing bytes')
        with self.assertRaisesRegex(RuntimeError, '덮어쓰지'):
            setup.ensure_model(self.root, opener=lambda *a, **k: self.fail('network must not run'))
        self.assertEqual(model.read_bytes(), b'keep existing bytes')

    def test_failed_download_cannot_become_ready_model(self):
        with patch.object(setup.time, 'sleep'), self.assertRaisesRegex(RuntimeError, '검증 실패'):
            setup.ensure_model(self.root, opener=lambda *a, **k: io.BytesIO(b'incomplete'))
        self.assertFalse((self.directory / 'mbra.onnx').exists())
        self.assertFalse((self.directory / '.setup-download-mbra.onnx').exists())

    def test_check_only_never_downloads(self):
        with self.assertRaisesRegex(RuntimeError, '모델이 없습니다'):
            setup.ensure_model(self.root, True, opener=lambda *a, **k: self.fail('network must not run'))

    def test_damaged_official_reference_is_rejected_before_download(self):
        (self.directory / 'inference.py').write_bytes(b'changed')
        with self.assertRaisesRegex(RuntimeError, '참조 파일'):
            setup.ensure_model(self.root, opener=lambda *a, **k: self.fail('network must not run'))

    def fake_venv(self):
        python = self.root / '.venv_mbra/bin/python'
        python.parent.mkdir(parents=True)
        python.write_text('keep me')
        return python

    def test_existing_matching_environment_does_not_run_installer(self):
        python = self.fake_venv()
        with patch.object(setup, 'dependency_versions', return_value=({'numpy': '2'}, {'numpy': '2'})), \
                patch.object(setup, 'run', side_effect=AssertionError('must not install')):
            self.assertEqual(setup.ensure_venv(self.root), {'numpy': '2'})
        self.assertEqual(python.read_text(), 'keep me')

    def test_existing_mismatched_environment_is_preserved(self):
        python = self.fake_venv()
        with patch.object(setup, 'dependency_versions', return_value=({'numpy': '2'}, {'numpy': '1'})), \
                patch.object(setup, 'run', side_effect=AssertionError('must not install')), \
                self.assertRaisesRegex(RuntimeError, '수정하지'):
            setup.ensure_venv(self.root)
        self.assertEqual(python.read_text(), 'keep me')

    def test_completed_environment_retires_interrupted_setup_marker(self):
        python = self.fake_venv()
        marker = self.root / 'runtime/setup/venv_incomplete.json'
        setup.save_report(marker, {'directory': str(self.root / '.venv_mbra'),
                                   'created_by': 'setup_environment.py'})
        with patch.object(setup, 'dependency_versions', return_value=({'numpy': '2'}, {'numpy': '2'})), \
                patch.object(setup, 'run', side_effect=AssertionError('must not install')):
            setup.ensure_venv(self.root)
        self.assertFalse(marker.exists())
        self.assertEqual(python.read_text(), 'keep me')

    def test_missing_ros_fails_with_prerequisites(self):
        with patch.object(setup, 'ROS_SETUP', self.root / 'missing'), \
                self.assertRaisesRegex(RuntimeError, 'sudo apt install'):
            setup.source_environment(self.root)

    def test_missing_ensurepip_is_detected_before_model_download(self):
        with patch.object(setup.shutil, 'which', return_value='/usr/bin/tool'), \
                patch.object(setup, 'run', side_effect=subprocess.CalledProcessError(1, ['python'])), \
                self.assertRaisesRegex(RuntimeError, 'python3-venv'):
            setup.prerequisites(self.root, {})

    def test_interrupted_new_environment_can_resume_without_touching_other_environment(self):
        calls = []
        installed = False
        install_calls = 0
        def fake_run(command, **kwargs):
            nonlocal installed, install_calls
            calls.append(command)
            if 'venv' in command:
                python = self.root / '.venv_mbra/bin/python'
                python.parent.mkdir(parents=True, exist_ok=True)
                python.write_text('new environment only')
            if 'install' in command:
                install_calls += 1
                if install_calls == 1:
                    raise subprocess.CalledProcessError(1, command)
                installed = True
        def versions(*args):
            return {'numpy': '2'}, {'numpy': '2' if installed else None}
        with patch.object(setup, 'run', side_effect=fake_run), \
                patch.object(setup, 'dependency_versions', side_effect=versions):
            with self.assertRaises(subprocess.CalledProcessError):
                setup.ensure_venv(self.root)
            self.assertTrue((self.root / 'runtime/setup/venv_incomplete.json').exists())
            self.assertEqual(setup.ensure_venv(self.root), {'numpy': '2'})
        self.assertFalse((self.root / 'runtime/setup/venv_incomplete.json').exists())
        self.assertTrue(any('--only-binary=:all:' in call for call in calls))
        self.assertTrue(all(str(self.root / '.venv_mbra') == call[-1]
                            for call in calls if 'venv' in call))

    def test_setup_does_not_touch_running_experiment(self):
        runtime = self.root / 'runtime'
        runtime.mkdir()
        with (runtime / 'control.lock').open('a') as held:
            fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.object(setup, 'ROOT', self.root), patch.object(sys, 'argv', ['setup.sh']), \
                    patch.object(setup, 'source_environment') as source:
                self.assertEqual(setup.main(), 1)
                source.assert_not_called()
        self.assertEqual(json.loads((runtime / 'setup/report.json').read_text())['status'], 'failed')

    def test_foreign_plugin_hash_selects_new_local_folder_without_modifying_results(self):
        tools = self.root / 'tools'
        tools.mkdir()
        (tools / 'run_experiments.py').write_text(
            "def fingerprint(): return {'schema':2, 'contact_plugin_sha256':'new-machine', 'files':{'a':'same'}}\n")
        results = self.root / 'results'
        results.mkdir()
        manifest = results / 'experiment.json'
        manifest.write_text(json.dumps({'schema': 2, 'contact_plugin_sha256': 'original', 'files': {'a': 'same'}}))
        csv = results / 'swan_on.csv'
        csv.write_text('keep results')
        before = manifest.read_bytes()
        actual=setup.check_protocol(self.root)
        from wheelchair_ai_driver.result_paths import select_results
        selected=select_results(self.root,actual)
        self.assertNotEqual(selected,results)
        self.assertFalse(selected.exists())  # setup only previews; first start creates it.
        self.assertEqual(manifest.read_bytes(), before)
        self.assertEqual(csv.read_text(), 'keep results')

    def test_matching_fingerprint_does_not_rewrite_manifest(self):
        tools = self.root / 'tools'
        tools.mkdir()
        (tools / 'run_experiments.py').write_text("def fingerprint(): return {'schema':2}\n")
        results = self.root / 'results'
        results.mkdir()
        manifest = results / 'experiment.json'
        manifest.write_text('{"schema": 2}\n')
        before = manifest.stat().st_mtime_ns
        self.assertEqual(setup.check_protocol(self.root), {'schema': 2})
        self.assertEqual(manifest.stat().st_mtime_ns, before)


if __name__ == '__main__':
    unittest.main()
