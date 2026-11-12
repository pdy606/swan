import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from wheelchair_ai_driver.observation import Pose2D
from wheelchair_ai_driver.trajectory_control import PathFollower, ReverseRecovery
from wheelchair_ai_driver.derived_world import generate, canonical


class DrivingTests(unittest.TestCase):
    def test_curvature_speed_slew_and_no_pivot(self):
        pose = Pose2D(0, 0, 0, 0)
        follower = PathFollower()
        straight = [(i*0.3, 0.0) for i in range(1, 9)]
        v, w, _ = follower.command(straight, pose, straight, 10, .05)
        self.assertAlmostEqual(v, .0075)
        self.assertEqual(w, 0)
        for _ in range(100):
            v, w, _ = follower.command(straight, pose, straight, 10, .05)
        self.assertLessEqual(v, .35)
        curved = [(i*.2, i*.2) for i in range(1, 9)]
        for _ in range(100):
            v2, w2, data = follower.command(curved, pose, curved, 10, .05)
        self.assertLess(v2, v)
        self.assertGreater(w2, 0)
        self.assertLessEqual(abs(data['steer_rad']), math.radians(35))
        self.assertLessEqual(abs(w2), .45)
        v, w, _ = follower.command([(-1., 0.)]*8, pose, straight, 10, .05)
        self.assertEqual((v, w), (0, 0))

    def test_current_pose_transform_and_invalid_data(self):
        path = [(1+i*.2, 2.) for i in range(8)]
        follower = PathFollower()
        _, w, _ = follower.command(path, Pose2D(0, 1, 2, 0), path, 10, .05)
        self.assertEqual(w, 0)
        with self.assertRaises(ValueError):
            follower.command([(float('nan'),0)]*8, Pose2D(0,0,0,0), path, 10, .05)

    def test_reverse_distance_and_additional_leg_after_zero_window(self):
        recovery = ReverseRecovery(Pose2D(0, 0, 0, 0), 0, 0)
        self.assertEqual(recovery.step(Pose2D(.2,0,0,0),.2,.2)[1], 0)
        self.assertEqual(recovery.step(Pose2D(.4,0,0,0),.4,.4)[1], -.1)
        self.assertEqual(recovery.step(Pose2D(1,-.1,0,0),1,1)[1], -.1)
        self.assertEqual(recovery.step(Pose2D(2.5,-.201,0,0),2.5,2.5)[:2], ('WAIT_RECOVERED',0))
        self.assertEqual(recovery.step(Pose2D(3.6,-.201,0,0),3.6,3.6)[:2], ('WAIT_RECOVERED',0))
        self.assertEqual(recovery.step(Pose2D(3.8,-.201,0,0),3.8,3.8)[:2], ('REVERSING',0))
        self.assertAlmostEqual(recovery.target,.30)
        self.assertLess(recovery.step(Pose2D(3.85,-.201,0,0),3.85,3.85)[1],0)

    def test_reverse_stall_and_bad_odom(self):
        recovery = ReverseRecovery(Pose2D(0,0,0,0),0,0)
        recovery.step(Pose2D(.4,0,0,0),.4,.4)
        self.assertEqual(recovery.step(Pose2D(2,0,0,0),2,2)[0], 'FAILED')
        recovery = ReverseRecovery(Pose2D(0,0,0,0),0,0)
        self.assertEqual(recovery.step(Pose2D(.4,-1,0,0),.4,.4)[0], 'FAILED')

    def test_derived_sdf_preserves_all_original_semantics(self):
        root = Path(__file__).resolve().parents[1]
        world = generate(root)
        original = ET.parse(root.parent/'wheelchair_gazebo/models/wheelchair/model.sdf').getroot()
        derived = ET.parse(world.parent/'wheelchair_driver/model.sdf').getroot()
        base = derived.find("./model/link[@name='base_link']")
        camera = base.find("sensor[@name='camera_driver_sensor']")
        self.assertEqual(camera.findtext('topic'), '/camera_driver')
        self.assertEqual(camera.findtext('camera/image/width'), '320')
        base.remove(camera)
        self.assertEqual(canonical(original), canonical(derived))


if __name__ == '__main__':
    unittest.main()
