"""Real ROS node, mocked worker/service transport; run in an isolated ROS domain."""
import math
import threading
import time
import unittest
from unittest.mock import Mock, patch

try:
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.task import Future
    from std_msgs.msg import String
    from std_srvs.srv import SetBool, Trigger
    from wheelchair_ai_driver.driver_node import MbraDriver
    from wheelchair_ai_driver.observation import Pose2D
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'Source ROS2 setup.bash for state integration tests')
class DriverStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-p', 'assistance_enabled:=true', '-p', 'use_sim_time:=true'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        worker = Mock()
        worker.ready = threading.Event()
        worker.ready.set()
        worker.busy = False
        worker.poll.return_value = []
        self.worker_patch = patch('wheelchair_ai_driver.preview_node.InferenceClient', return_value=worker)
        self.worker_patch.start()
        self.node = MbraDriver()
        self.node.now_seconds = Mock(return_value=100.0)
        self.node.control_sim = 99.95
        self.node.odom.append(Pose2D(100, 22, 0, 0))
        self.node.image_wall = self.node.odom_wall = time.monotonic()
        self.node.autonomy = Mock()
        self.node.autonomy.call_async.side_effect = lambda req: Future()
        self.node.run_started = self.node.progress_time = 100.0
        self.node.progress_pose = self.node.odom.poses[-1]
        self.node.state = 'DRIVING'

    def tearDown(self):
        self.node.destroy_node()
        self.worker_patch.stop()

    def status(self, value):
        message = String()
        message.data = value
        self.node.assistance_callback(message)

    def test_every_nav_owner_state_discards_pending_and_outputs_zero(self):
        for status in ('STOPPED','WAITING_FOR_NAV','AVOIDING','REPLANNING','VERIFYING_CLEAR'):
            self.node.state, self.node.paused = 'DRIVING', False
            generation = self.node.generation
            self.node.pending = {'id': 1, 'generation': generation}
            self.node.latest_result = {'old': True}
            self.status(status)
            self.assertEqual(self.node.state, 'ASSISTING')
            self.assertTrue(self.node.paused)
            self.assertIsNone(self.node.latest_result)
            self.assertEqual(self.node.last_command, (0,0))
            dropped = self.node.dropped_results
            self.node.handle_worker_result({'type':'result','id':1,'generation':generation})
            self.assertEqual(self.node.dropped_results, dropped+1)
        self.status('COMPLETED')
        self.assertEqual(self.node.state, 'REARMING')
        self.node.now_seconds.return_value = 100.41
        self.node.control_sim = 100.4
        self.node.odom.append(Pose2D(100.41,22,0,0))
        self.node.control_tick()
        self.assertEqual(self.node.state, 'WAITING')
        self.assertFalse(self.node.paused)
        self.assertEqual(self.node.last_command, (0,0))
        self.assertIsNone(self.node.latest_result)

    def test_repeated_blocked_sessions_require_recovered_and_have_fresh_budgets(self):
        self.assertEqual(self.node.value('max_recovery_attempts'),0)
        for number in range(1,5):
            self.node.now_seconds.return_value=100.+number*10
            self.status('BLOCKED')
            r=self.node.recovery
            self.assertIsNotNone(r)
            self.assertEqual(self.node.recovery_attempts,number)
            self.assertEqual(r.cumulative_reverse_m,0.)
            self.assertEqual(r.summary(self.node.now_seconds())['elapsed_sim_sec'],0.)
            self.status('BLOCKED')  # Duplicate notification is still the same session.
            self.assertIs(self.node.recovery,r)
            self.assertEqual(self.node.recovery_attempts,number)
            self.status('COMPLETED')  # Only the original BLOCKED release permits resuming.
            self.assertIs(self.node.recovery,r)
            self.node.latest_result={'stale':True}
            self.status('RECOVERED')
            self.assertEqual(self.node.state,'REARMING')
            self.assertEqual(self.node.last_command,(0,0))
            self.assertIsNone(self.node.latest_result)
            self.assertIsNone(self.node.recovery)
        self.assertEqual(len(self.node.recovery_history),4)

    def test_optional_positive_recovery_cap_and_missing_odom_still_stop(self):
        original=self.node.value
        self.node.value=lambda name:1 if name=='max_recovery_attempts' else original(name)
        self.status('BLOCKED');self.status('RECOVERED');self.status('BLOCKED')
        self.assertEqual(self.node.state,'FAILED')
        self.assertEqual(self.node.last_command,(0,0))

    def test_late_mode_ack_cannot_restart_emergency_stopped_driver(self):
        self.node.state = 'DISABLED'
        self.node.need_off = False
        self.node.ready_to_start = Mock(return_value=True)
        self.node.count_publishers = Mock(return_value=1)
        future = Future()
        self.node.autonomy.call_async.side_effect = [future, Future()]
        self.node.start_callback(Trigger.Request(),Trigger.Response())
        self.assertEqual(self.node.state,'ARMING')
        self.node.stop('ESTOP','test')
        ack = SetBool.Response()
        ack.success = True
        future.set_result(ack)
        self.assertEqual(self.node.state,'ESTOP')
        self.assertEqual(self.node.last_command,(0,0))

    def test_recovered_preempts_active_reverse_and_discards_old_prediction(self):
        self.status('BLOCKED')
        r=self.node.recovery
        r.step(Pose2D(100.4,22,0,0),100.4,time.monotonic())
        self.node.publish_command(-.1)
        self.node.latest_result={'stale':True}
        self.status('RECOVERED')
        self.assertEqual(self.node.last_command,(0,0))
        self.assertIsNone(self.node.recovery)
        self.assertEqual(self.node.state,'REARMING')
        self.assertTrue(self.node.paused)
        self.assertIsNone(self.node.latest_result)
        self.assertEqual(self.node.recovery_history[-1]['outcome'],'RECOVERED')

    def test_recovery_total_deadline_stops_and_records_failure(self):
        self.status('BLOCKED')
        self.node.now_seconds.return_value=115.01
        self.node.control_sim=114.96
        self.node.odom.append(Pose2D(115.01,22,0,0))
        self.node.odom_wall=self.node.image_wall=time.monotonic()
        self.node.control_tick()
        self.assertEqual(self.node.state,'FAILED')
        self.assertEqual(self.node.last_command,(0,0))
        self.assertEqual(self.node.recovery_history[-1]['reason'],'recovery_sim_timeout')

    def test_blind_reverse_is_refused_without_simulation_time(self):
        original=self.node.value
        self.node.value=lambda name:False if name=='use_sim_time' else original(name)
        self.status('BLOCKED')
        self.assertEqual(self.node.state,'FAILED')
        self.assertEqual(self.node.reason,'blind_reverse_requires_simulation_time')
        self.assertEqual(self.node.last_command,(0,0))
        self.assertIsNone(self.node.recovery)

    def test_additional_reverse_then_recovered_returns_to_fresh_inference(self):
        self.status('BLOCKED')
        def tick(stamp,x):
            self.node.now_seconds.return_value=stamp
            self.node.control_sim=stamp-.05
            self.node.odom.append(Pose2D(stamp,x,0,0))
            self.node.odom_wall=self.node.image_wall=time.monotonic()
            self.node.control_tick()
        tick(100.4,22.)
        self.assertLess(self.node.last_command[0],0)
        tick(102.5,21.8)
        self.assertEqual(self.node.state,'WAIT_RECOVERED')
        self.assertEqual(self.node.last_command,(0,0))
        tick(103.6,21.8)
        self.assertEqual(self.node.state,'WAIT_RECOVERED')
        tick(103.8,21.8)
        tick(103.85,21.8)
        self.assertEqual(self.node.state,'REVERSING')
        self.assertLess(self.node.last_command[0],0)
        self.node.latest_result={'stale':True}
        self.status('RECOVERED')
        self.assertEqual(self.node.last_command,(0,0))
        self.assertEqual(self.node.recovery_history[-1]['additional_reverses'],1)
        tick(104.26,21.8)
        self.assertEqual(self.node.state,'WAITING')
        self.assertFalse(self.node.paused)
        self.assertIsNone(self.node.latest_result)

    def test_blocked_event_during_arming_is_not_lost(self):
        self.node.state = 'DISABLED'
        self.node.need_off = False
        self.node.ready_to_start = Mock(return_value=True)
        self.node.count_publishers = Mock(return_value=1)
        future = Future()
        self.node.autonomy.call_async.side_effect = [future, Future()]
        self.node.start_callback(Trigger.Request(),Trigger.Response())
        self.status('BLOCKED')
        ack = SetBool.Response()
        ack.success = True
        future.set_result(ack)
        self.assertEqual(self.node.state,'SETTLING')
        self.assertTrue(self.node.paused)

    def test_stale_sensor_stops_and_requests_real_autonomy_off(self):
        self.node.odom_wall = time.monotonic()-3
        self.node.control_tick()
        self.assertEqual(self.node.state,'FAILED')
        self.assertEqual(self.node.last_command,(0,0))
        request = self.node.autonomy.call_async.call_args.args[0]
        self.assertFalse(request.data)

    def test_runtime_mode_parameter_cannot_silently_change_experiment_condition(self):
        result = self.node.set_parameters([Parameter('assistance_enabled',value=False)])[0]
        self.assertFalse(result.successful)
        self.assertTrue(self.node.value('assistance_enabled'))


if __name__ == '__main__':
    unittest.main()
