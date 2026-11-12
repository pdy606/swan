import math
import unittest
from wheelchair_ai_driver.observation_metrics import MotionMetrics,route_distance


class ObservationMetricsTests(unittest.TestCase):
    def test_integrates_speed_and_path_without_counting_teleport(self):
        metrics=MotionMetrics()
        metrics.add(1,0,0,.2,0)
        metrics.add(2,.2,0,.2,.1)
        metrics.add(3,.4,0,0,0)
        result=metrics.summary()
        self.assertAlmostEqual(result['odom_path_length_m'],.4)
        self.assertAlmostEqual(result['mean_abs_speed_mps'],.15)
        self.assertEqual(result['max_abs_speed_mps'],.2)
        self.assertFalse(metrics.add(3,.4,0,0,0))
        metrics.add(3.05,10,0,0,0)
        self.assertEqual(metrics.odom_jumps,1)
        self.assertAlmostEqual(metrics.distance,.4)

    def test_route_distance_includes_original_spawn_segment(self):
        route=[[-45.78,0],[-35,0],[20.5,0],[30,2.2],[53,0]]
        self.assertEqual(route_distance((-45,0),route),0)
        self.assertAlmostEqual(route_distance((-40,1.2),route),1.2)
        self.assertEqual(route_distance((30,2.2),route),0)


if __name__=='__main__':
    unittest.main()
