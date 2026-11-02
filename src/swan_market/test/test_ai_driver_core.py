"""Checks for the AI operator's independent speed and clearance gate."""

import importlib.util
from pathlib import Path
import sys
import unittest


path = Path(__file__).resolve().parents[1] / 'scripts/ai_driver_core.py'
spec = importlib.util.spec_from_file_location('ai_driver_core', path)
core = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = core
spec.loader.exec_module(core)


class AIDriverCoreTest(unittest.TestCase):
    def test_clearances_reject_nan_and_use_range_max_for_infinity(self):
        scan = [1.2, float('nan'), float('inf'), 0.7, 2.0]
        clear = core.scan_clearances(scan, -1.0, 0.5, 8.0)
        self.assertAlmostEqual(clear['front'], 8.0)
        self.assertAlmostEqual(clear['left'], 0.7)
        self.assertAlmostEqual(clear['right'], 1.2)

    def test_hard_stop_cannot_be_overridden_by_model(self):
        clear = {'front': 0.8, 'left': 2.0, 'right': 2.0}
        self.assertEqual(core.bounded_action('forward', clear, 3.0),
                         ('stop', 'front_clearance'))
        clear['front'] = 2.0
        self.assertEqual(core.bounded_action('forward', clear, 0.2),
                         ('stop', 'goal_reached'))
        self.assertEqual(core.bounded_action('teleport', clear, 3.0),
                         ('stop', 'invalid_action'))

    def test_fixed_speed_contract(self):
        self.assertEqual(set(core.ACTIONS), {'stop', 'forward', 'left', 'right'})
        self.assertLessEqual(max(abs(v[0]) for v in core.ACTIONS.values()), 0.12)
        self.assertLessEqual(max(abs(v[1]) for v in core.ACTIONS.values()), 0.35)


if __name__ == '__main__':
    unittest.main()
