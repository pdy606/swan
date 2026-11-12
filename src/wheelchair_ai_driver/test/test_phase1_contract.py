import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PhaseOneContractTests(unittest.TestCase):
    def test_ros_executable_has_only_preview_publishers_and_no_service_clients(self):
        tree = ast.parse((ROOT/'wheelchair_ai_driver/preview_node.py').read_text())
        topics = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotEqual(node.func.attr, 'create_client')
                if node.func.attr == 'create_publisher':
                    self.assertIsInstance(node.args[1], ast.Constant)
                    topics.append(node.args[1].value)
        self.assertEqual(sorted(topics), ['/ai_driver/inference_status', '/ai_driver/predicted_path'])

    def test_worker_cannot_receive_extra_sensor_inputs_through_ros(self):
        worker_source = (ROOT/'wheelchair_ai_driver/inference_worker.py').read_text()
        self.assertNotIn('rclpy', worker_source)
        self.assertNotIn('LaserScan', worker_source)
        self.assertNotIn('Twist', worker_source)


if __name__ == '__main__':
    unittest.main()
