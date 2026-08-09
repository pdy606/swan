#!/usr/bin/env python3
"""
[LiDAR 노이즈] /scan(팀원 깨끗한 스캔) → 가우시안 노이즈 + 드롭아웃 → /scan_noisy
목적: 실제 LiDAR(RPLIDAR)처럼 거리에 잡음/결측을 넣어 sim-to-real 갭을 줄인다.
      팀원 모델 SDF 를 수정하지 않고 '우리 쪽'에서 센서 리얼리즘 추가.

파라미터:
  range_stddev   거리 노이즈 표준편차 [m]  (RPLIDAR A1 ≈ 0.01~0.03)
  dropout_prob   점 결측 확률 (반사 실패 흉내)
  enable         false 면 그대로 통과 (A/B 비교용)

사용: lidar_perception 이 /scan_noisy 를 구독하게 하면 노이즈 반영됨.
"""
import random
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanNoise(Node):
    def __init__(self):
        super().__init__('scan_noise_node')
        self.declare_parameter('in_topic', '/scan')
        self.declare_parameter('out_topic', '/scan_noisy')
        self.declare_parameter('range_stddev', 0.02)   # 2cm (RPLIDAR A1 급)
        self.declare_parameter('dropout_prob', 0.02)   # 2% 점 결측
        self.declare_parameter('enable', True)
        g = lambda n: self.get_parameter(n).value
        self.sigma = g('range_stddev')
        self.dropout = g('dropout_prob')
        self.enable = g('enable')

        self.pub = self.create_publisher(LaserScan, g('out_topic'), 10)
        self.create_subscription(LaserScan, g('in_topic'), self.on_scan, 10)
        self.get_logger().info(
            f'scan_noise 시작: σ={self.sigma}m, dropout={self.dropout*100:.0f}%, '
            f'{"ON" if self.enable else "통과(OFF)"}')

    def on_scan(self, msg: LaserScan):
        if not self.enable:
            self.pub.publish(msg)
            return
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = msg.intensities
        noisy = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                noisy.append(r)
                continue
            if random.random() < self.dropout:
                noisy.append(float('inf'))          # 반사 실패 = 결측
            else:
                noisy.append(r + random.gauss(0.0, self.sigma))   # 거리 잡음
        out.ranges = noisy
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(ScanNoise())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
