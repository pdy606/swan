#!/usr/bin/env python3
"""
[휠체어 전용 조종기] 실제 전동휠체어 조이스틱처럼 조작.
 - 속도는 '유지'된다 (키를 계속 안 눌러도 됨) = 스로틀 방식
 - 조향은 놓으면 서서히 중앙 복원 (조이스틱처럼)
 - 어시스트 상태를 실시간 표시 → 시스템이 언제 개입하는지 눈에 보임

실행: ros2 run swan_pipeline wheelchair_teleop
"""
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

HELP = """
┌─────────────────────────────────────────────────────────┐
│  SWAN 휠체어 조종기                                      │
│                                                         │
│   w / s   속도 ±0.2 m/s  (유지됨)                        │
│   a / d   좌 / 우 조향   (놓으면 자동 복원)               │
│   1~4     속도 프리셋 (0.5 / 1.0 / 1.4 / 1.67)          │
│  space    즉시 정지                                      │
│    x      조향 중앙                                      │
│    q      종료                                           │
└─────────────────────────────────────────────────────────┘
"""

V_MAX, V_STEP = 1.67, 0.2       # 보도 법정 6km/h
W_MAX, W_STEP = 1.0, 0.3
PRESETS = {'1': 0.5, '2': 1.0, '3': 1.4, '4': 1.67}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class WheelchairTeleop(Node):
    def __init__(self):
        super().__init__('wheelchair_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel_user', 10)
        self.create_subscription(String, '/swan/state', self.on_state, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_actual, 10)
        self.v = self.w = 0.0
        self.state = '-'
        self.actual_v = 0.0
        self.create_timer(0.05, self.tick)      # 20 Hz 로 계속 발행

    def on_state(self, m: String):
        self.state = m.data

    def on_actual(self, m: Twist):
        self.actual_v = m.linear.x

    def key(self, k):
        if k == 'w':   self.v = clamp(self.v + V_STEP, 0.0, V_MAX)
        elif k == 's': self.v = clamp(self.v - V_STEP, 0.0, V_MAX)
        elif k == 'a': self.w = clamp(self.w + W_STEP, -W_MAX, W_MAX)
        elif k == 'd': self.w = clamp(self.w - W_STEP, -W_MAX, W_MAX)
        elif k == 'x': self.w = 0.0
        elif k == ' ': self.v = self.w = 0.0
        elif k in PRESETS: self.v = PRESETS[k]

    def tick(self):
        # 조향 자동 중앙복원 (조이스틱 손 떼면 돌아오듯)
        self.w *= 0.88
        if abs(self.w) < 0.02:
            self.w = 0.0
        m = Twist()
        m.linear.x, m.angular.z = self.v, self.w
        self.pub.publish(m)

        # 상태 한 줄 표시 (시스템 개입이 보이게)
        mark = '  ← 시스템 개입중!' if self.state not in ('수동주행', '-') else ''
        sys.stdout.write(
            f"\r 내 요청 {self.v:4.2f} m/s | 조향 {self.w:+4.2f} | "
            f"실제 {self.actual_v:4.2f} m/s | 상태: {self.state:<6s}{mark}        ")
        sys.stdout.flush()


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = WheelchairTeleop()
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k == 'q':
                    break
                node.key(k)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        m = Twist()
        node.pub.publish(m)          # 종료 시 정지
        node.destroy_node()
        rclpy.shutdown()
        print("\n조종 종료")


if __name__ == '__main__':
    main()
