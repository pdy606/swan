"""Background control tests use isolated test records, never Gazebo or real CSVs."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('experiment_cli',ROOT/'tools/experiment_cli.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
spec2=importlib.util.spec_from_file_location('batch_runner',ROOT/'tools/run_experiments.py')
runner=importlib.util.module_from_spec(spec2);spec2.loader.exec_module(runner)


class ExperimentControlTests(unittest.TestCase):
    def setUp(self):
        self.local_runner=unittest.mock.Mock()
        self.local_runner.fingerprint.return_value={'schema':2,'test_only':True}
        loader=patch.object(cli,'runner_module',return_value=self.local_runner)
        loader.start();self.addCleanup(loader.stop)

    def wait_for(self,predicate,timeout=8):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:time.sleep(.025)
        self.assertTrue(predicate())

    def test_background_stop_resume_asymmetric_totals_and_automatic_exit(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'runtime') as temp:
            root=Path(temp);(root/'runtime').mkdir();(root/'results').mkdir()
            # A test batch emits synthetic data only in this temporary directory.
            # It records the safe-stop signal before exiting without a commit.
            fake=root/'fake_batch.py'
            fake.write_text('''import sys,time,signal
from pathlib import Path
import json
sys.path.insert(0,sys.argv[1])
from wheelchair_ai_driver.experiment_store import ExperimentStore
from wheelchair_ai_driver.result_paths import active_results
r=Path(sys.argv[2]);targets={'off':int(sys.argv[3]),'on':int(sys.argv[4])}
def stop(*args):
 (r/'safe_stop').touch();sys.exit(0)
signal.signal(signal.SIGINT,stop)
folder=active_results(r)
s=ExperimentStore(folder,json.loads((folder/'experiment.json').read_text()))
for number in range(1,max(targets.values())+1):
 for mode in ('off','on'):
  if number>targets[mode] or s.completed(mode,number):continue
  (r/'active').touch()
  while not (r/'finish_allowed').exists():time.sleep(.025)
  time.sleep(.05)
  s.commit(mode,number,{'collisions':1,'progress_percent':50.,'sim_seconds':1.,'reason':'test_failure','avoidance_interventions':2})
''')
            entry=root/'test_worker.py'
            entry.write_text(f'''import sys
from pathlib import Path
sys.path.insert(0,{str(ROOT/'tools')!r})
import experiment_cli as c
a=sys.argv
targets={{'off':int(a[a.index('--off')+1]),'on':int(a[a.index('--on')+1])}}
root=Path({str(root)!r})
command=[sys.executable,{str(fake)!r},{str(ROOT)!r},str(root),str(targets['off']),str(targets['on'])]
sys.exit(c.worker(root,targets,a[a.index('--token')+1],command=command))
''')
            command=[sys.executable,str(entry),'_worker'];child=None
            try:
                child=cli.start(root,{'off':1,'on':2},command)
                self.wait_for(lambda:(root/'active').exists())
                with self.assertRaises(RuntimeError):cli.start(root,{'off':2,'on':2},command)
                state=cli.read_json(root/'runtime/experiment_state.json')
                self.assertTrue(cli.alive(state))
                with patch.object(cli,'stop_orphans'):
                    cli.stop(root)
                child.wait(timeout=3)
                self.assertTrue((root/'safe_stop').exists())
                self.assertEqual(cli.completed(root),{'off':0,'on':0})
                (root/'finish_allowed').touch()
                child=cli.start(root,{'off':1,'on':2},command)
                self.assertEqual(child.wait(timeout=8),0)
                self.assertEqual(cli.completed(root),{'off':1,'on':2})
                self.assertEqual(cli.read_json(root/'runtime/experiment_state.json')['status'],'complete')
                self.assertFalse(cli.alive(cli.read_json(root/'runtime/experiment_state.json')))
                # Failed test trials are committed and included in the average.
                self.assertIn('평균,1.000000,50.000000,1.000000,2.000000',(root/'results/swan_on.csv').read_text())
                with patch.object(cli.subprocess,'Popen',side_effect=AssertionError('must not launch')):
                    self.assertIsNone(cli.start(root,{'off':1,'on':2},command))
                # A changed environment gets a fresh series even though the old
                # target is complete. Returning to the old environment resumes it.
                original=cli.active_results(root)
                old_csv=(original/'swan_on.csv').read_bytes()
                self.local_runner.fingerprint.return_value={'schema':2,'test_only':True,'environment':'B'}
                child=cli.start(root,{'off':1,'on':1},command)
                self.assertEqual(child.wait(timeout=8),0)
                self.assertNotEqual(cli.active_results(root),original)
                self.assertEqual(cli.completed(root),{'off':1,'on':1})
                self.assertEqual((original/'swan_on.csv').read_bytes(),old_csv)
                self.local_runner.fingerprint.return_value={'schema':2,'test_only':True}
                with patch.object(cli.subprocess,'Popen',side_effect=AssertionError('must reuse old results')):
                    self.assertIsNone(cli.start(root,{'off':1,'on':2},command))
                self.assertEqual(cli.active_results(root),original)
            finally:
                if child and child.poll() is None:child.terminate();child.wait(timeout=5)

    def test_pid_reuse_and_stale_pid_are_not_signalled(self):
        state={'pid':os.getpid(),'process_identity':'not-the-current-process','token':'never'}
        self.assertFalse(cli.alive(state))
        self.assertFalse(cli.alive({'pid':999999999,'process_identity':'0','token':'never'}))

    def test_missing_trial_identity_is_not_hidden_by_total_count(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'runtime') as temp:
            root=Path(temp);records=root/'results/records';records.mkdir(parents=True)
            for name in ('off_001.json','off_003.json','on_001.json'):(records/name).write_text('{}')
            self.assertEqual(cli.completed(root)['off'],2)
            self.assertFalse(cli.targets_met(root,{'off':2,'on':1}))

    def test_tagged_cleanup_leaves_unrelated_process_running(self):
        marker='swan_cleanup_test_'+str(os.getpid())
        owned=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],env=dict(os.environ,GZ_PARTITION=marker),stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        other=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'])
        try:
            self.wait_for(lambda:owned.pid in runner.own_processes(marker))
            self.assertEqual(runner.cleanup(marker),[])
            owned.wait(timeout=2)
            self.assertIsNone(other.poll())
        finally:
            for process in (owned,other):
                if process.poll() is None:process.terminate()
                process.wait(timeout=3)


if __name__=='__main__':unittest.main()
