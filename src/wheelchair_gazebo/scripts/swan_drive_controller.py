#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


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

        # Preserve the existing high-level nonzero/zero fold gate. The user
        # stops the wheelchair before folding. No navigation control is added.
        self.fold_request = None
        self.fold_plan = []
        self.fold_index = 0
        self.fold_targets = {}
        self.fold_active = False
        self.fold_step_started = None
        self.fold_settled_since = None
        self.fold_warned = False
        self.joint_positions = {}
        self.joint_feedback_time = None
        self.joint_feedback_serial = 0
        self.fold_checked_serial = -1
        self.fold_publishers = {
            name: self.create_publisher(
                Float64, '/model/wheelchair/fold/' + slug + '/cmd_pos', 10)
            for name, slug in [
                ('camera_fold_joint', 'camera'),
                ('front_cover_fold_joint', 'cover'),
                ('front_wheel_fold_joint', 'wheel'),
                ('swan_fold_joint', 'main'),
            ]
        }
        self.fold_joint_names = tuple(self.fold_publishers) + ('front_steering_joint',)
        self.joint_subscription = self.create_subscription(
            JointState,
            '/world/swan_test_world/model/wheelchair/joint_state',
            self.joint_state_callback, 10)
        self.fold_timer = self.create_timer(0.05, self.fold_timer_callback)
        self.folded = False
        self.fold_sub = self.create_subscription(
            Float64,
            '/model/wheelchair/fold/cmd_pos',
            self.fold_callback,
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
        # During storage the sequencer owns steering, including the exact
        # 90-degree target; the normal driving clamp remains unchanged.
        if self.folded or self.fold_active:
            steer_msg.data = self.fold_targets.get('front_steering_joint', 0.0)
        self.steer_pub.publish(steer_msg)

    def joint_state_callback(self, msg: JointState):
        # Scoped joint names are accepted without changing existing SDF names.
        positions = {name.rsplit('::', 1)[-1]: value
                     for name, value in zip(msg.name, msg.position)
                     if math.isfinite(value)}
        if all(name in positions for name in self.fold_joint_names):
            self.joint_positions = positions
            self.joint_feedback_time = self.get_clock().now().nanoseconds / 1e9
            self.joint_feedback_serial += 1

    def fold_callback(self, msg: Float64):
        if not math.isfinite(msg.data):
            self.get_logger().warning('Ignoring non-finite fold command')
            return
        requested = (msg.data != 0.0)
        self.folded = requested  # Existing command-based gate, unchanged.
        # Repeated terminal publications must not restart the current step.
        if self.fold_request == requested:
            return
        self.fold_request = requested
        self.fold_active = True
        self.fold_targets = {}
        self.fold_index = 0
        self.fold_step_started = None
        self.fold_settled_since = None
        self.fold_warned = False
        angle = math.pi / 2.0
        self.fold_plan = [
            ('camera down', {'camera_fold_joint': angle}),
            ('steering and cover aligned', {'front_steering_joint': angle,
                                             'front_cover_fold_joint': angle}),
            ('wheel stacked', {'front_wheel_fold_joint': angle}),
            ('package upright', {'swan_fold_joint': angle}),
        ] if requested else [
            ('package lowered', {'swan_fold_joint': 0.0}),
            ('wheel deployed', {'front_wheel_fold_joint': 0.0}),
            ('steering and cover centred', {'front_steering_joint': 0.0,
                                            'front_cover_fold_joint': 0.0}),
            ('camera upright', {'camera_fold_joint': 0.0}),
        ]
        # Preserve the existing stop-on-command behavior, retaining the current
        # steering position until fresh feedback initializes the sequence.
        self.fold_targets['front_steering_joint'] = self.joint_positions.get(
            'front_steering_joint', 0.0)
        self.publish_commands(wheel_speed=0.0, steering=0.0)
        self.get_logger().info('Folding requested' if requested else 'Unfolding requested')

    def fold_position_tolerance(self, name, target):
        # On unfolding the grounded steering wheel can retain a small static
        # error under the existing PID. This is already a driving configuration:
        # allow 3 degrees around centre before raising the camera. Keep sending
        # the exact zero target; do not relax wheel/cover/main/camera hinges or
        # the 90-degree alignment required before folding the wheel.
        if (self.fold_request is False and self.fold_index >= 2
                and name == 'front_steering_joint' and abs(target) < 1e-9):
            return math.radians(3.0)
        return 0.018

    def fold_timer_callback(self):
        if self.fold_request is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        fresh = (self.joint_feedback_time is not None
                 and 0.0 <= now - self.joint_feedback_time < 0.5)
        if self.fold_active and self.fold_step_started is None:
            if not fresh:
                if not self.fold_warned:
                    self.get_logger().warning('Folding waits for joint feedback; check joint_state bridge')
                    self.fold_warned = True
                return
            # On direction reversal, hold the measured position of every other
            # hinge; never jump back to a guessed endpoint.
            if len(self.fold_targets) < len(self.fold_joint_names):
                self.fold_targets = {name: self.joint_positions[name]
                                     for name in self.fold_joint_names}
            label, targets = self.fold_plan[self.fold_index]
            self.fold_targets.update(targets)
            self.fold_step_started = now
            self.fold_settled_since = None
            self.fold_warned = False
            self.fold_checked_serial = self.joint_feedback_serial
            self.get_logger().info(f'Fold step {self.fold_index + 1}/4: {label}')

        # Republish internal targets continuously, so startup discovery and
        # single dropped samples cannot leave a hinge uncommanded.
        for name, pub in self.fold_publishers.items():
            if name in self.fold_targets:
                pub.publish(Float64(data=self.fold_targets[name]))
        if self.fold_active or self.folded:
            self.publish_commands(wheel_speed=0.0, steering=0.0)
        if not self.fold_active:
            return
        if not fresh:
            self.fold_settled_since = None
        elif self.joint_feedback_serial != self.fold_checked_serial:
            self.fold_checked_serial = self.joint_feedback_serial
            # Require every held joint to remain within its tolerance for
            # 0.3 seconds of fresh simulation-time feedback.
            arrived = all(abs(self.joint_positions[name] - target)
                          < self.fold_position_tolerance(name, target)
                          for name, target in self.fold_targets.items())
            if arrived:
                if self.fold_settled_since is None:
                    self.fold_settled_since = now
                elif now - self.fold_settled_since >= 0.3:
                    self.fold_index += 1
                    self.fold_step_started = None
                    self.fold_settled_since = None
                    if self.fold_index == len(self.fold_plan):
                        self.fold_active = False
                        self.last_cmd_time = self.get_clock().now()
                        self.get_logger().info(
                            'FOLD COMPLETE' if self.folded else 'UNFOLD COMPLETE')
                    return
            else:
                self.fold_settled_since = None
        if now - self.fold_step_started > 20.0 and not self.fold_warned:
            label = self.fold_plan[self.fold_index][0]
            if not fresh:
                details = 'joint feedback missing/stale'
            else:
                details = '; '.join(
                    f'{name}: actual={self.joint_positions[name]:.4f}, '
                    f'target={target:.4f}, '
                    f'error={self.joint_positions[name] - target:+.4f} rad, '
                    f'tolerance={self.fold_position_tolerance(name, target):.4f}'
                    for name, target in self.fold_targets.items()
                    if abs(self.joint_positions[name] - target)
                    >= self.fold_position_tolerance(name, target)
                ) or 'positions have not stayed settled for 0.3 seconds'
            self.get_logger().warning(
                f'Fold step {self.fold_index + 1}/4 ({label}) still waiting: '
                f'{details}. Holding this step; not skipping the sequence.')
            self.fold_warned = True

    def cmd_callback(self, msg: Twist):

        if self.folded or self.fold_active:
            self.publish_commands(wheel_speed=0.0, steering=0.0)
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

        if self.folded or self.fold_active:
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
