#!/usr/bin/env python3
"""
[제어 노드] ★ SWAN 차별점 모듈 ★ — 다른 팀 자산과 독립적으로 갈아끼울 수 있게 설계.

역할: 목표속도를 받아 Jerk(가속도 변화율)를 제한하며 부드럽게 추종 → Twist 발행.
      brake_level 이 높을수록 더 센 감속/Jerk 허용 (다단 제동).

[통합 포인트]
  input_mode:=drive_target  → /swan/drive_target (swan_interfaces/DriveTarget) 구독
  input_mode:=twist         → 외부(Nav2 등)가 내는 Twist 를 구독해 '스무딩'만 수행
      예) Nav2 → /cmd_vel_nav → [이 노드] → /cmd_vel → 로봇
  모든 제어 상수는 ROS 파라미터 (YAML 튜닝 가능)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from swan_interfaces.msg import DriveTarget


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('a_max', 1.5)          # 최대 가/감속 [m/s^2]
        self.declare_parameter('j_max', 2.5)          # 최대 jerk [m/s^3] ← 승차감 핵심
        self.declare_parameter('omega_rate', 2.0)     # 각속도 변화 제한 [rad/s^2]
        self.declare_parameter('brake_a_gain', 0.7)   # brake_level 당 a_max 증가율
        self.declare_parameter('brake_j_gain', 1.0)   # brake_level 당 j_max 증가율
        self.declare_parameter('input_mode', 'drive_target')   # drive_target | twist
        self.declare_parameter('input_topic', '/swan/drive_target')
        self.declare_parameter('output_topic', '/cmd_vel')

        g = lambda n: self.get_parameter(n).value
        self.a_max0, self.j_max0 = g('a_max'), g('j_max')
        self.omega_rate = g('omega_rate')
        self.bag, self.bjg = g('brake_a_gain'), g('brake_j_gain')
        self.dt = 1.0 / g('rate_hz')
        mode, in_topic, out_topic = g('input_mode'), g('input_topic'), g('output_topic')

        self.pub = self.create_publisher(Twist, out_topic, 10)
        if mode == 'twist':
            self.create_subscription(Twist, in_topic, self.on_twist, 10)
        else:
            self.create_subscription(DriveTarget, in_topic, self.on_target, 10)

        self.v_cmd = self.a_cmd = self.omega_cmd = 0.0
        self.v_target = self.omega_target = 0.0
        self.brake_level = 0
        self.create_timer(self.dt, self.loop)
        self.get_logger().info(
            f'control 시작: mode={mode}, in={in_topic}, out={out_topic}, '
            f'a_max={self.a_max0}, j_max={self.j_max0}')

    # 입력 A: 우리 판단 노드
    def on_target(self, m: DriveTarget):
        self.v_target, self.omega_target = m.v_target, m.omega_target
        self.brake_level = m.brake_level

    # 입력 B: 외부 Twist(Nav2 등) → 스무딩만 수행
    def on_twist(self, m: Twist):
        self.v_target, self.omega_target = m.linear.x, m.angular.z
        self.brake_level = 0

    def loop(self):
        dt = self.dt
        a_max = self.a_max0 * (1.0 + self.bag * self.brake_level)
        j_max = self.j_max0 * (1.0 + self.bjg * self.brake_level)

        # ── S-curve 속도 서보 (오버슛 방지) ──
        # 현재 가속도를 jerk 한계로 0까지 되돌리는 동안 속도가 '추가로' 변하는 양을
        # 미리 빼고 판단해야 목표를 지나치지 않는다.
        err = self.v_target - self.v_cmd
        v_extra = self.a_cmd * abs(self.a_cmd) / (2.0 * j_max)
        margin = err - v_extra
        a_des = a_max if margin > 1e-6 else (-a_max if margin < -1e-6 else 0.0)

        self.a_cmd = clamp(a_des, self.a_cmd - j_max * dt, self.a_cmd + j_max * dt)
        self.a_cmd = clamp(self.a_cmd, -a_max, a_max)
        self.v_cmd = max(0.0, self.v_cmd + self.a_cmd * dt)
        if abs(err) < 1e-3 and abs(self.a_cmd) < 1e-3:
            self.v_cmd, self.a_cmd = self.v_target, 0.0

        d = clamp(self.omega_target - self.omega_cmd,
                  -self.omega_rate * dt, self.omega_rate * dt)
        self.omega_cmd += d

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self.v_cmd, self.omega_cmd
        self.pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(ControlNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
