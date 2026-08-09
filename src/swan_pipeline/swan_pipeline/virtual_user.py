#!/usr/bin/env python3
"""
[가상 사용자] 데모용 — 사람이 키보드를 잡고 '계속 전진'하는 상황을 흉내낸다.
실제 조작으로 바꾸려면 이 노드 대신 teleop 을 /cmd_vel_user 로 remap 해서 쓰면 됨:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_user
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VirtualUser(Node):
    def __init__(self):
        super().__init__('virtual_user')
        self.declare_parameter('v', 1.67)      # 사용자가 원하는 속도 (6 km/h)
        self.declare_parameter('w', 0.0)
        self.v = self.get_parameter('v').value
        self.w = self.get_parameter('w').value
        self.pub = self.create_publisher(Twist, '/cmd_vel_user', 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'virtual_user: 사용자가 {self.v} m/s 로 계속 전진한다고 가정')

    def tick(self):
        m = Twist()
        m.linear.x, m.angular.z = self.v, self.w
        self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(VirtualUser())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
