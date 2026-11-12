import math
import unittest

from wheelchair_ai_driver.observation import FrameBuffer, OdomBuffer, Pose2D, quaternion_yaw
from wheelchair_ai_driver.waypoints import RouteTracker, relative_goal, world_to_odom


class GeometryContextTests(unittest.TestCase):
    def test_six_frames_span_one_second_without_reusing_frames(self):
        buffer = FrameBuffer()
        for index in range(11):
            buffer.append({'stamp': index / 10})
        self.assertEqual([round(frame['stamp'], 2) for frame in buffer.context()],
                         [0, 0.2, 0.4, 0.6, 0.8, 1.0])
        self.assertFalse(buffer.append({'stamp': 1.0}))

    def test_missing_history_is_not_fabricated(self):
        buffer = FrameBuffer()
        for stamp in [0, 0.1, 0.2, 0.3, 0.7, 0.8, 0.9, 1.0]:
            buffer.append({'stamp': stamp})
        self.assertIsNone(buffer.context())

    def test_odom_interpolates_through_wrapped_yaw(self):
        buffer = OdomBuffer()
        buffer.append(Pose2D(0, 0, 2, math.radians(179)))
        buffer.append(Pose2D(0.1, 2, 4, math.radians(-179)))
        pose = buffer.pose_at(0.05)
        self.assertAlmostEqual(pose.x, 1)
        self.assertAlmostEqual(pose.y, 3)
        self.assertAlmostEqual(abs(pose.yaw), math.pi)
        self.assertIsNone(buffer.pose_at(1.0))

    def test_zero_quaternion_is_rejected(self):
        with self.assertRaises(ValueError):
            quaternion_yaw(0, 0, 0, 0)

    def test_goal_signs_and_nonidentity_world_transform(self):
        pose = Pose2D(0, 10, 20, math.pi / 2)
        forward, left = relative_goal((8, 23), pose)
        self.assertAlmostEqual(forward, 3)
        self.assertAlmostEqual(left, 2)
        transformed = world_to_odom((1, 0), (10, 20), math.pi / 2)
        self.assertAlmostEqual(transformed[0], 10)
        self.assertAlmostEqual(transformed[1], 21)

    def test_route_lookahead_and_goal_completion(self):
        route = RouteTracker([(1, 0), (10, 0)], lookahead=3)
        temporary, reached = route.target(Pose2D(0, 0.3, 0, 0))
        self.assertEqual(route.index, 1)
        self.assertAlmostEqual(temporary[0], 3.3)
        self.assertFalse(reached)
        _, reached = route.target(Pose2D(1, 9.7, 0, 0))
        self.assertTrue(reached)


if __name__ == '__main__':
    unittest.main()
