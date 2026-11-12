"""Recovery distance/time budgets and preemption; no rear sensor assumptions."""
import math
import unittest

from wheelchair_ai_driver.observation import Pose2D
from wheelchair_ai_driver.trajectory_control import ReverseRecovery


class StagedRecoveryTests(unittest.TestCase):
    def test_all_legs_stop_and_wait_and_distance_bound(self):
        r=ReverseRecovery(Pose2D(0,0,0,0),0,0)
        t=x=0.; v=0.; stops=[]
        for _ in range(500):
            t+=.05; x+=v*.05
            phase,v,reason=r.step(Pose2D(t,x,0,0),t,t*4)
            if phase=='WAIT_RECOVERED' and (not stops or len(r.legs)>len(stops)):
                stops.append((t,-x))
            if phase=='FAILED':break
        self.assertEqual(reason,'recovery_distance_limit_without_recovered')
        self.assertEqual(len(stops),4)
        for stop,target in zip(stops,(.2,.3,.4,.5)):
            self.assertAlmostEqual(stop[1],target,delta=.006)
        self.assertLessEqual(r.cumulative_reverse_m,.50)
        self.assertLess(t,15)
        self.assertEqual(r.summary(t)['additional_reverses'],3)
        for a,b in zip(r.legs,r.legs[1:]):
            self.assertGreaterEqual(b['start_sim']-a['stop_sim'],1.2-1e-9)

    def test_timeout_is_one_total_simulation_budget_not_wait_or_wall_budget(self):
        r=ReverseRecovery(Pose2D(100,0,0,0),100,1000)
        self.assertEqual(r.step(Pose2D(100.4,0,0,0),100.4,1040)[0],'REVERSING')
        phase,v,reason=r.step(Pose2D(115,0,0,0),115,1080)
        self.assertEqual((phase,v,reason),('FAILED',0.,'recovery_sim_timeout'))

    def test_odom_distance_jump_lateral_yaw_and_stall_are_bounded(self):
        for p in (Pose2D(.4,-.51,0,0),Pose2D(.4,-.1,.11,0),Pose2D(.4,-.1,0,.26),
                  Pose2D(.4,.07,0,0),Pose2D(.4,float('nan'),0,0)):
            r=ReverseRecovery(Pose2D(0,0,0,0),0,0)
            self.assertEqual(r.step(p,.4,.4)[0],'FAILED')
        r=ReverseRecovery(Pose2D(0,0,0,0),0,0)
        r.step(Pose2D(.4,0,0,0),.4,.4)
        self.assertEqual(r.step(Pose2D(2,0,0,0),2,2)[2],'reverse_stalled_or_timeout')

    def test_zero_wait_does_not_reuse_old_six_second_failure(self):
        r=ReverseRecovery(Pose2D(0,0,0,0),0,0)
        r.step(Pose2D(.4,0,0,0),.4,.4)
        r.step(Pose2D(2.5,-.2,0,0),2.5,2.5)
        self.assertEqual(r.phase,'WAIT_RECOVERED')
        self.assertEqual(r.step(Pose2D(8,-.2,0,0),8,8)[:2],('REVERSING',0.))
        self.assertAlmostEqual(r.target,.30)

    def test_yaw_aligned_reverse_uses_original_frame(self):
        r=ReverseRecovery(Pose2D(0,3,4,math.pi/2),0,0)
        r.step(Pose2D(.4,3,4,math.pi/2),.4,.4)
        phase,v,_=r.step(Pose2D(2.5,3,3.8,math.pi/2),2.5,2.5)
        self.assertEqual((phase,v),('WAIT_RECOVERED',0.))
        self.assertAlmostEqual(r.backward,.2)

    def test_paused_simulation_never_keeps_reverse_command_queued(self):
        r=ReverseRecovery(Pose2D(0,0,0,0),0,0)
        self.assertLess(r.step(Pose2D(.4,0,0,0),.4,.4)[1],0)
        self.assertEqual(r.step(Pose2D(.4,0,0,0),.4,.5)[1],0)
