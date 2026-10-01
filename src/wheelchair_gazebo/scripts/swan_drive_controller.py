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

        # 조향 모터는 포크 위끝에 있고, 바퀴는 그 앞에 달린 스윙암 끝에 있다.
        #   큰바퀴 축 x=-0.13 -> 조향축 x=0.51 : pivot_offset
        #   조향축 -> 바퀴 축                   : arm_length
        self.pivot_offset = 0.64
        self.arm_length = 0.17

        # 바퀴가 발판 쪽으로 휩쓸려 들어오지 않는 한계 (model.sdf 관절 한계 1.06 rad)
        self.max_steer = math.radians(60.0)

        # 제자리 회전 대신: 최대로 꺾고 천천히 전진하는 최소 반경 회전
        self.pivot_steer = self.max_steer
        # 최소 반경 회전 시 큰바퀴 축 중심의 최대 전진 속도 [m/s]
        self.pivot_linear_speed = 0.25

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

    def solve_leading_arm(self, v, w):
        """
        v, w : 큰바퀴 축 중심의 전진 속도 [m/s], 요 각속도 [rad/s]

        접지점 P = (b + a cos d, a sin d) (큰바퀴 축 기준)
        P 의 속도 (v - w a sin d, w (b + a cos d)) 가 바퀴 방향 (cos d, sin d)
        과 평행하려면  v sin d - w b cos d = w a
        """
        a = self.arm_length
        b = self.pivot_offset

        # 후진은 같은 식을 |v| 와 부호 바꾼 w 로 푼다.
        direction = 1.0 if v >= 0.0 else -1.0
        speed = abs(v)
        yaw = direction * w

        r = math.hypot(speed, yaw * b)
        phi = math.atan2(yaw * b, speed)
        steering = phi + math.asin(self.clamp(yaw * a / r, -1.0, 1.0))

        steering = self.clamp(
            steering,
            -self.max_steer,
            self.max_steer
        )

        # 접지점의 실제 속도 = 바퀴 선속도
        vx = v - w * a * math.sin(steering)
        vy = w * (b + a * math.cos(steering))

        return steering, direction * math.hypot(vx, vy)

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
        # 바퀴가 조향축 앞에 달려 있어 제자리 회전은 불가능하다.
        # 최대 조향각으로 꺾고, 요청한 각속도가 나오는 만큼만
        # 천천히 전진하는 최소 반경 회전으로 대신한다.
        # --------------------------------------------------

        if abs(v) < self.linear_deadband:

            a = self.arm_length
            b = self.pivot_offset

            creep = abs(w) * (
                a + b * math.cos(self.pivot_steer)
            ) / math.sin(self.pivot_steer)

            creep = min(creep, self.pivot_linear_speed)

            steering, wheel_linear_speed = self.solve_leading_arm(creep, w)

        # --------------------------------------------------
        # 직진 / 곡선 주행
        #
        # 바퀴 접지점이 조향축 앞 arm_length 에 있으므로
        # 일반 bicycle model 대신, 접지점 속도가 바퀴 방향과
        # 일치하는 조향각을 직접 푼다.
        # --------------------------------------------------

        else:

            steering, wheel_linear_speed = self.solve_leading_arm(v, w)

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