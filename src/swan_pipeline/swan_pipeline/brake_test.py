#!/usr/bin/env python3
"""
[제동 실험 시나리오]  4초간 주행 후 정지 명령. 두 방식 비교용.
  mode:=abrupt  → /cmd_vel 을 즉시 0 (기존 급정지, control_node 안 거침)
  mode:=scurve  → /swan/drive_target v_target=0 (control_node의 Jerk 제한 감속)
사용: ros2 run swan_pipeline brake_test --ros-args -p mode:=abrupt
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from swan_interfaces.msg import DriveTarget

V_RUN = 1.67      # 실제 전동휠체어 보도 최고속 (6 km/h 법정)
T_BRAKE = 4.0      # 이 시점에 정지 명령


class BrakeTest(Node):
    def __init__(self):
        super().__init__('brake_test')
        self.declare_parameter('mode', 'abrupt')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        if self.mode == 'abrupt':
            self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        else:
            self.pub = self.create_publisher(DriveTarget, '/swan/drive_target', 10)
        self.t0 = self.get_clock().now()
        self.create_timer(0.02, self.tick)   # 50 Hz
        self.get_logger().info(f'brake_test 시작 (mode={self.mode})')

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        v = V_RUN if t < T_BRAKE else 0.0
        if self.mode == 'abrupt':
            m = Twist()
            m.linear.x = float(v)          # 4초에 1.2 → 0 으로 '뚝'
            self.pub.publish(m)
        else:
            m = DriveTarget()
            m.header.stamp = self.get_clock().now().to_msg()
            m.mode = DriveTarget.MODE_NORMAL if v > 0 else DriveTarget.MODE_BRAKE
            m.v_target = float(v)
            m.omega_target = 0.0
            m.brake_level = 0              # 0 = 승차감 우선 (Jerk 최소)
            self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(BrakeTest())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
