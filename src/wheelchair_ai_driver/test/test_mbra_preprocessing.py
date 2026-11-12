import base64
import unittest

import numpy as np

from wheelchair_ai_driver.mbra_adapter import decode_frame, goal_to_pose, preprocess_frames


class MbraPreprocessingTests(unittest.TestCase):
    def test_goal_uses_cos_before_sin_and_metric_spacing(self):
        np.testing.assert_allclose(goal_to_pose([0, 0.8]), [[0, 1, 0, 1]], atol=1e-6)
        np.testing.assert_allclose(goal_to_pose([0.8, 0]), [[1, 0, 1, 0]], atol=1e-6)
        with self.assertRaises(ValueError):
            goal_to_pose([float('nan'), 1])

    def test_bgr_padding_is_not_read_as_pixels(self):
        frame = {'width': 1, 'height': 1, 'step': 5, 'encoding': 'bgr8',
                 'data': base64.b64encode(bytes([0, 0, 255, 123, 124])).decode()}
        np.testing.assert_array_equal(decode_frame(frame), [[[255, 0, 0]]])
        tensor = preprocess_frames([frame] * 6)
        self.assertEqual(tensor.shape, (1, 6, 3, 96, 96))
        self.assertEqual(tensor.dtype, np.float32)
        np.testing.assert_allclose(tensor[0, 0, :, 0, 0],
                                   [(1-.485)/.229, (0-.456)/.224, (0-.406)/.225], rtol=1e-6)

    def test_bad_image_size_and_wrong_context_are_rejected(self):
        frame = {'width': 1, 'height': 1, 'step': 3, 'encoding': 'rgb8', 'data': ''}
        with self.assertRaises(ValueError):
            decode_frame(frame)
        with self.assertRaises(ValueError):
            preprocess_frames([frame] * 5)

    def test_chronological_channel_order_is_preserved(self):
        frames = [{'width': 1, 'height': 1, 'step': 3, 'encoding': 'rgb8',
                   'data': base64.b64encode(bytes([index * 30, 0, 0])).decode()}
                  for index in range(6)]
        tensor = preprocess_frames(frames)
        self.assertTrue(np.all(np.diff(tensor[0, :, 0, 0, 0]) > 0))


if __name__ == '__main__':
    unittest.main()
