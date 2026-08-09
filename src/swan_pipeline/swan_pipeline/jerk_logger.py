#!/usr/bin/env python3
"""
[메트릭 로거] 실제 주행 데이터를 CSV로 기록 → 나중에 가속도·Jerk 계산/그래프.
/odom = 로봇의 실제 속도(물리 결과), /cmd_vel = 명령 속도.
사용: ros2 run swan_pipeline jerk_logger --ros-args -p output:=/tmp/run.csv
"""
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class JerkLogger(Node):
    def __init__(self):
        super().__init__('jerk_logger')
        self.declare_parameter('output', '/tmp/jerk_run.csv')
        path = self.get_parameter('output').get_parameter_value().string_value
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'v_odom', 'v_cmd'])
        self.v_cmd = 0.0
        self.t0 = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.get_logger().info(f'jerk_logger 기록 시작 → {path}')

    def on_cmd(self, m: Twist):
        self.v_cmd = m.linear.x

    def on_odom(self, m: Odometry):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        self.w.writerow([f'{now - self.t0:.4f}',
                         f'{m.twist.twist.linear.x:.6f}',
                         f'{self.v_cmd:.6f}'])
        self.f.flush()


def main():
    rclpy.init()
    node = JerkLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
