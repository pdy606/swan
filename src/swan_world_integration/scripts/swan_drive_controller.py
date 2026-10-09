#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class SwanDriveController(Node):

    def __init__(self):
        super().__init__('swan_drive_controller')

        # model.sdf 기준
        self.wheel_radius = 0.20

        # 휠체어 큰바퀴 축 x=-0.13
        # SWAN 조향축 x=0.79
        self.wheelbase = 0.92

        # 완전 90도에서는 수치적으로 너무 극단적이므로
        # 일반 최대 조향은 약 88도
        self.max_steer = math.radians(88.0)

        # 제자리 회전 비슷한 동작
        self.pivot_steer = math.radians(85.0)
        self.pivot_linear_speed = 0.12

        self.linear_deadband = 0.02
        self.angular_deadband = 0.02

        # 앞바퀴 최대 회전속도 [rad/s]
        self.max_wheel_speed = 12.0

        # 방향 반대면 이것만 -1.0으로 변경
        self.drive_sign = 1.0
        self.steer_sign = 1.0

        self.drive_pub = self.create_publisher(
            Float64,
            '/model/wheelchair/front_drive/cmd_vel',
            10
        )

        self.steer_pub = self.create_publisher(
            Float64,
            '/model/wheelchair/front_steering/cmd_pos',
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/model/wheelchair/cmd_vel',
            self.cmd_callback,
            10
        )

        self.last_cmd_time = self.get_clock().now()

        # 명령이 끊겼을 경우 자동 정지
        self.watchdog_timer = self.create_timer(
            0.1,
            self.watchdog_callback
        )

        self.get_logger().info(
            'SWAN drive controller started'
        )

    def clamp(self, value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def publish_commands(self, wheel_speed, steering):
        drive_msg = Float64()
        drive_msg.data = (
            self.drive_sign *
            self.clamp(
                wheel_speed,
                -self.max_wheel_speed,
                self.max_wheel_speed
            )
        )

        steer_msg = Float64()
        steer_msg.data = (
            self.steer_sign *
            self.clamp(
                steering,
                -self.max_steer,
                self.max_steer
            )
        )

        self.drive_pub.publish(drive_msg)
        self.steer_pub.publish(steer_msg)

    def cmd_callback(self, msg: Twist):

        self.last_cmd_time = self.get_clock().now()

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # 완전 정지
        if (
            abs(v) < self.linear_deadband and
            abs(w) < self.angular_deadband
        ):
            self.publish_commands(
                wheel_speed=0.0,
                steering=0.0
            )
            return

        # --------------------------------------------------
        # angular.z만 들어온 경우
        #
        # 기존 DiffDrive는 여기서 제자리 회전을 했지만
        # SWAN은 앞바퀴를 약 85도로 꺾고 아주 천천히
        # 굴려서 pseudo-pivot 동작을 만든다.
        # --------------------------------------------------

        if abs(v) < self.linear_deadband:

            steering = math.copysign(
                self.pivot_steer,
                w
            )

            # angular 값에 따라 살짝 속도 조절
            strength = self.clamp(
                abs(w),
                0.35,
                1.0
            )

            wheel_linear_speed = (
                self.pivot_linear_speed *
                strength
            )

        # --------------------------------------------------
        # 직진
        # --------------------------------------------------

        elif abs(w) < self.angular_deadband:

            steering = 0.0
            wheel_linear_speed = v

        # --------------------------------------------------
        # 일반 곡선 주행
        #
        # bicycle model:
        #
        # tan(delta) = L * w / v
        # --------------------------------------------------

        else:

            steering = math.atan(
                (self.wheelbase * w) / v
            )

            steering = self.clamp(
                steering,
                -self.max_steer,
                self.max_steer
            )

            # 조향할수록 앞바퀴 진행방향이 기울기 때문에
            # 지나친 보상은 하지 않고 최대 약 1.6배까지만.
            cos_delta = abs(math.cos(steering))

            speed_scale = 1.0 / max(
                cos_delta,
                0.625
            )

            wheel_linear_speed = (
                v * speed_scale
            )

        # m/s -> wheel rad/s
        wheel_speed = (
            wheel_linear_speed /
            self.wheel_radius
        )

        self.publish_commands(
            wheel_speed,
            steering
        )

    def watchdog_callback(self):

        elapsed = (
            self.get_clock().now() -
            self.last_cmd_time
        ).nanoseconds / 1e9

        if elapsed > 0.7:
            self.publish_commands(
                wheel_speed=0.0,
                steering=0.0
            )


def main(args=None):

    rclpy.init(args=args)

    node = SwanDriveController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_commands(
            wheel_speed=0.0,
            steering=0.0
        )

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()