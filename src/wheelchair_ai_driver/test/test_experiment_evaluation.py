import csv
import json
from pathlib import Path
import tempfile
import unittest
from wheelchair_ai_driver.experiment_metrics import CourseProgress
from wheelchair_ai_driver.avoidance_metrics import AvoidanceInterventions
from wheelchair_ai_driver.experiment_store import ExperimentStore,completed_attempt,save_json

ROOT=Path(__file__).resolve().parents[1]

class EvaluationTests(unittest.TestCase):
    def test_accepted_avoidance_counts_once_across_replans_and_waits(self):
        c=AvoidanceInterventions()
        for s in ('STOPPED','WAITING_FOR_NAV','REPLANNING'):
            self.assertFalse(c.observe(s))
        self.assertTrue(c.observe('AVOIDING'))
        for s in ('AVOIDING','WAITING_FOR_NAV','REPLANNING','WAITING_FOR_NAV','AVOIDING','VERIFYING_CLEAR','AVOIDING'):
            self.assertFalse(c.observe(s))
        self.assertEqual(c.count,1)
        self.assertFalse(c.observe('COMPLETED'))
        self.assertTrue(c.observe('AVOIDING'))
        self.assertEqual(c.count,2)

    def test_blocked_ends_episode_and_reverse_is_not_an_intervention(self):
        c=AvoidanceInterventions()
        c.observe('AVOIDING');c.observe('BLOCKED')
        for s in ('BLOCKED','RECOVERED','STOPPED','WAITING_FOR_NAV'):
            self.assertFalse(c.observe(s))
        self.assertEqual(c.count,1)
        self.assertTrue(c.observe('AVOIDING'))
        c.observe('SLOPE_MANUAL');self.assertTrue(c.observe('AVOIDING'))
        self.assertEqual(c.count,3)
        off=AvoidanceInterventions(False)
        self.assertFalse(off.observe('AVOIDING'));self.assertEqual(off.count,0)

    def test_on_csv_has_interventions_off_and_legacy_schema_are_unchanged(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'runtime') as folder:
            store=ExperimentStore(folder,{'schema':2})
            failed={'collisions':1,'progress_percent':40.,'sim_seconds':100.,'reason':'failed','avoidance_interventions':3}
            store.commit('on',1,failed)
            store.commit('on',2,dict(failed,avoidance_interventions=5,reason='arrived',progress_percent=100.))
            store.commit('off',1,failed)
            with self.assertRaises(ValueError):store.commit('on',3,{k:v for k,v in failed.items() if k!='avoidance_interventions'})
            rows=list(csv.reader((Path(folder)/'swan_on.csv').open(encoding='utf-8-sig')))
            self.assertEqual(rows[0][-1],'자동 회피 개입 횟수')
            self.assertEqual(rows[1][-1],'3')
            self.assertEqual(rows[-1],['평균','1.000000','70.000000','100.000000','4.000000'])
            rows_off=list(csv.reader((Path(folder)/'swan_off.csv').open(encoding='utf-8-sig')))
            self.assertTrue(all(len(r)==4 for r in rows_off))
            store=ExperimentStore(folder,{'schema':2})
            self.assertTrue(store.completed('on',2))
        with tempfile.TemporaryDirectory(dir=ROOT/'runtime') as folder:
            store=ExperimentStore(folder,{'schema':1})
            store.commit('on',1,{'collisions':0,'progress_percent':100.,'sim_seconds':20.})
            rows=list(csv.reader((Path(folder)/'swan_on.csv').open(encoding='utf-8-sig')))
            self.assertTrue(all(len(r)==4 for r in rows))

    def test_reversing_and_looping_do_not_inflate_progress(self):
        c=CourseProgress((0,0),[(10,0),(20,0)])
        self.assertAlmostEqual(c.update((5,0)),25)
        for point in [(3,0),(3,1),(5,1),(5,0)]:self.assertEqual(c.update(point),25)
        self.assertAlmostEqual(c.update((7,0)),35)

    def test_skip_detour_and_premature_arrival_are_rejected(self):
        c=CourseProgress((0,0),[(10,0),(10,10),(0,10)])
        self.assertLess(c.update((0,10),arrived=True),100)
        self.assertEqual(c.segment,0)
        self.assertEqual(c.update((8,5)),0)
        c.update((10,0));c.update((10,10))
        self.assertEqual(c.update((0,10),arrived=True),100)

    def test_route_length_and_goal_tolerance(self):
        c=CourseProgress((0,0),[(3,0),(3,4)])
        self.assertEqual(c.total,7)
        self.assertAlmostEqual(c.update((1.5,0)),100*1.5/7)
        c.update((3,0));self.assertLess(c.update((3,3.6)),100)
        self.assertEqual(c.update((3,3.6),arrived=True),100)

    def test_interrupted_attempt_is_preserved_and_not_counted_as_failure(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'runtime') as folder:
            directory=Path(folder)
            save_json(directory/'result.json',{'reason':'operator_interrupted','collisions':4})
            self.assertIsNone(completed_attempt(directory))
            self.assertFalse((directory/'result.json').exists())
            self.assertEqual(json.loads((directory/'interrupted_result.json').read_text())['collisions'],4)
            self.assertIsNone(completed_attempt(directory))
            save_json(directory/'result.json',{'reason':'simulation_timeout','collisions':4})
            self.assertEqual(completed_attempt(directory)['reason'],'simulation_timeout')

    def test_durable_resume_duplicate_guard_and_failed_trial_average(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'runtime') as folder:
            store=ExperimentStore(folder,{'same':True})
            r={'collisions':2,'progress_percent':20.,'sim_seconds':10.,'reason':'failed'}
            store.commit('off',1,r)
            store=ExperimentStore(folder,{'same':True})
            self.assertTrue(store.completed('off',1))
            with self.assertRaises(ValueError):store.commit('off',1,r)
            store.commit('off',2,dict(r,collisions=0,progress_percent=100.,sim_seconds=30.))
            rows=list(csv.reader((Path(folder)/'swan_off.csv').open(encoding='utf-8-sig')))
            self.assertEqual(rows[-1],['평균','1.000000','60.000000','20.000000'])
            self.assertEqual(len(rows),4)
            # Simulated interruption after durable JSON but before CSV refresh.
            (Path(folder)/'swan_off.csv').write_text('partial')
            store.refresh()
            self.assertEqual(len(list(csv.reader((Path(folder)/'swan_off.csv').open(encoding='utf-8-sig')))),4)
            with self.assertRaises(ValueError):ExperimentStore(folder,{'same':False})

if __name__=='__main__':unittest.main()
