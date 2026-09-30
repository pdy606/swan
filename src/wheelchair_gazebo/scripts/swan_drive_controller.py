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
        self.wheel_radius = 0.18

        # 휠체어 큰바퀴 축 x=-0.13
        # SWAN 조향축 x=0.68
        self.wheelbase = 0.81

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

        # --------------------------------------------------
        # 수납(stow) 시퀀스
        #
        # /model/wheelchair/fold/cmd_pos
        #   0.0      -> 펼침 (주행 위치)
        #   그 외 값 -> 수납
        #
        # 수납: 1) 카메라 접기 + 구동 바퀴를 본체 아래로
        #       2) 본체(+바퀴)를 시트 아래로
        # 펼침은 역순.
        #
        # Gazebo 쪽은 단순 위치 PID라서, 속도와 순서는
        # 여기서 목표 위치를 조금씩 움직여 만든다.
        # --------------------------------------------------

        # model.sdf 관절 이동 범위
        self.retract_travel = 0.18
        self.slide_travel = 0.39
        self.camera_fold_angle = 1.5708

        self.retract_speed = 0.12   # [m/s]
        self.slide_speed = 0.15     # [m/s]
        self.camera_speed = 1.2     # [rad/s]

        self.folded = False

        # 현재 목표 위치 (0 = 펼침)
        self.retract_pos = 0.0
        self.slide_pos = 0.0
        self.camera_pos = 0.0
        self.stow_moving = False

        self.retract_pub = self.create_publisher(
            Float64,
            '/model/wheelchair/wheel_retract/cmd_pos',
            10
        )

        self.slide_pub = self.create_publisher(
            Float64,
            '/model/wheelchair/slide/cmd_pos',
            10
        )

        self.camera_fold_pub = self.create_publisher(
            Float64,
            '/model/wheelchair/camera_fold/cmd_pos',
            10
        )

        self.fold_sub = self.create_subscription(
            Float64,
            '/model/wheelchair/fold/cmd_pos',
            self.fold_callback,
            10
        )

        self.last_stow_time = None

        self.stow_timer = self.create_timer(
            0.02,
            self.stow_callback
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

    def fold_callback(self, msg: Float64):
        self.folded = (msg.data != 0.0)
        # Stop immediately on fold/unfold; never replay a pre-fold command.
        self.publish_commands(wheel_speed=0.0, steering=0.0)

    def drive_ready(self):
        # 완전히 펼쳐진 상태에서만 주행 허용
        return (
            not self.folded and
            self.retract_pos == 0.0 and
            self.slide_pos == 0.0
        )

    def step_toward(self, value, goal, max_step):
        # 도착 판정을 == 로 하므로 마지막 스텝은 goal 값을 그대로 반환
        if abs(goal - value) <= max_step:
            return goal

        return value + math.copysign(max_step, goal - value)

    def stow_callback(self):

        now = self.get_clock().now()

        if self.last_stow_time is None:
            self.last_stow_time = now
            return

        dt = (now - self.last_stow_time).nanoseconds / 1e9
        self.last_stow_time = now

        # 시뮬레이션 일시정지/시간 점프 보호
        dt = self.clamp(dt, 0.0, 0.1)

        if self.folded:
            retract_goal = self.retract_travel
            camera_goal = self.camera_fold_angle

            # 1단계가 끝난 뒤에만 본체를 넣는다
            stage1_done = (
                self.retract_pos == retract_goal and
                self.camera_pos == camera_goal
            )
            slide_goal = (
                self.slide_travel if stage1_done else self.slide_pos
            )

        else:
            slide_goal = 0.0

            # 본체가 다 나온 뒤에만 바퀴와 카메라를 편다
            body_out = (self.slide_pos == 0.0)
            retract_goal = 0.0 if body_out else self.retract_pos
            camera_goal = 0.0 if body_out else self.camera_pos

        kit_before = self.retract_pos + self.slide_pos
        camera_before = self.camera_pos

        self.retract_pos = self.step_toward(
            self.retract_pos,
            retract_goal,
            self.retract_speed * dt
        )
        self.slide_pos = self.step_toward(
            self.slide_pos,
            slide_goal,
            self.slide_speed * dt
        )
        self.camera_pos = self.step_toward(
            self.camera_pos,
            camera_goal,
            self.camera_speed * dt
        )

        for pub, value in (
            (self.retract_pub, self.retract_pos),
            (self.slide_pub, self.slide_pos),
            (self.camera_fold_pub, self.camera_pos),
        ):
            msg = Float64()
            msg.data = value
            pub.publish(msg)

        kit_moved = self.retract_pos + self.slide_pos - kit_before

        self.stow_moving = (
            kit_moved != 0.0 or
            self.camera_pos != camera_before
        )

        if kit_moved != 0.0 and dt > 0.0:
            # 바퀴가 휠체어 쪽으로 들어오는 속도만큼 바퀴를 굴려서
            # 휠체어는 제자리에 있고 구동기만 움직이게 한다.
            kit_speed = kit_moved / dt

            self.publish_commands(
                wheel_speed=-kit_speed / self.wheel_radius,
                steering=0.0
            )

    def cmd_callback(self, msg: Twist):

        if not self.drive_ready():
            return

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

        if not self.drive_ready():
            # 이동 중에는 stow_callback이 바퀴를 굴린다
            if not self.stow_moving:
                self.publish_commands(wheel_speed=0.0, steering=0.0)
            return

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