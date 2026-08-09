#!/usr/bin/env python3
"""
[시나리오 로거] 그래프용 데이터 기록.
매 스텝: 시간, 로봇(x,y,yaw), 실제속도, 명령속도, 장애물거리, 상태 → CSV

사용: ros2 run swan_pipeline scenario_logger --ros-args -p output:=/tmp/run.csv
"""
import csv
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from swan_interfaces.msg import DetectionArray


def yaw_from_quat(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class ScenarioLogger(Node):
    def __init__(self):
        super().__init__('scenario_logger')
        self.declare_parameter('output', '/tmp/scenario.csv')
        self.declare_parameter('cmd_topic', '/model/wheelchair/cmd_vel')
        path = self.get_parameter('output').value
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'yaw', 'v_actual', 'v_cmd', 'omega_cmd', 'obs_dist', 'state'])
        self.t0 = None
        self.x = self.y = self.yaw = self.v_act = 0.0
        self.v_cmd = self.w_cmd = 0.0
        self.obs = 99.0
        self.state = '-'
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Twist, self.get_parameter('cmd_topic').value, self.on_cmd, 10)
        self.create_subscription(DetectionArray, '/swan/detections', self.on_det, 10)
        self.create_subscription(String, '/swan/state', self.on_state, 10)
        self.create_timer(0.05, self.tick)   # 20 Hz 기록
        self.get_logger().info(f'scenario_logger → {path}')

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.x, self.y, self.yaw = p.x, p.y, yaw_from_quat(q.z, q.w)
        self.v_act = m.twist.twist.linear.x

    def on_cmd(self, m):
        self.v_cmd, self.w_cmd = m.linear.x, m.angular.z

    def on_det(self, m):
        self.obs = min([d.distance_m for d in m.detections], default=99.0)

    def on_state(self, m):
        self.state = m.data

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        self.w.writerow([f'{now-self.t0:.3f}', f'{self.x:.4f}', f'{self.y:.4f}', f'{self.yaw:.4f}',
                         f'{self.v_act:.4f}', f'{self.v_cmd:.4f}', f'{self.w_cmd:.4f}',
                         f'{self.obs:.3f}', self.state])
        self.f.flush()

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    rclpy.init()
    n = ScenarioLogger()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
