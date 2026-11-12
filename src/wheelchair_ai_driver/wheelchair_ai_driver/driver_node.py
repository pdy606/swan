"""Opt-in MBRA driver; the original SWAN supervisor remains final command owner."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.msg import SetParametersResult

from .preview_node import InferencePreview, ASSISTANCE_STATES
from .trajectory_control import ControlLimits, PathFollower, ReverseRecovery

TERMINAL = {'DISABLED', 'FAILED', 'ESTOP', 'ARRIVED'}


class MbraDriver(InferencePreview):
    def __init__(self):
        super().__init__('mbra_driver', {
            'camera_topic': '/camera_driver', 'result_file': 'runtime/reports/latest_driving_prediction.json',
            'max_speed_mps': 0.30, 'max_steer_deg': 35.0,
            'max_yaw_rate': 0.45, 'acceleration_mps2': 0.15, 'deceleration_mps2': 0.35,
            'tracking_lookahead_m': 0.65, 'assistance_enabled': False,
            'command_path_timeout_sec': 0.65, 'wall_sensor_timeout_sec': 2.0,
            'max_run_sim_sec': 600.0, 'max_recovery_attempts': 0,
        })
        self.follower = PathFollower(ControlLimits(
            max_speed=self.value('max_speed_mps'), max_steer=math.radians(self.value('max_steer_deg')),
            max_yaw_rate=self.value('max_yaw_rate'), acceleration=self.value('acceleration_mps2'),
            deceleration=self.value('deceleration_mps2'), lookahead=self.value('tracking_lookahead_m')))
        for name in ('command_path_timeout_sec', 'wall_sensor_timeout_sec', 'max_run_sim_sec'):
            if not math.isfinite(self.value(name)) or self.value(name) <= 0:
                raise ValueError(f'{name} must be positive')
        if self.value('max_recovery_attempts') < 0:
            raise ValueError('max_recovery_attempts must be nonnegative')
        self.state = 'DISABLED'
        self.reason = 'Call /ai_driver/start to enable motion'
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_user', 10)
        self.state_pub = self.create_publisher(String, '/ai_driver/driver_status', 10)
        self.prediction_pub = self.create_publisher(String, '/ai_driver/prediction', 10)
        self.actual_velocity = (0.0, 0.0)
        self.autonomy = self.create_client(SetBool, '/set_autonomy')
        self.start_service = self.create_service(Trigger, '/ai_driver/start', self.start_callback)
        self.stop_service = self.create_service(Trigger, '/ai_driver/stop', self.stop_callback)
        self.estop_service = self.create_service(Trigger, '/ai_driver/emergency_stop', self.estop_callback)
        self.service_epoch = 0
        self.need_off = True
        self.off_pending = False
        self.last_off_request = -math.inf
        self.service_deadline = None
        self.latest_prediction_wall = None
        self.image_wall = None
        self.odom_wall = None
        self.tick_wall = time.monotonic()
        self.control_sim = self.now_seconds()
        self.recovery = None
        self.recovery_attempts = 0
        self.recovery_history = []
        self.rearm_until = None
        self.run_started = None
        self.progress_pose = None
        self.progress_time = None
        self.last_command = (0.0, 0.0)
        self.tracking = {}
        self.last_pose = None
        self.odom_fault = None
        self.assist_started = None
        self.state_history = []
        self.arming_status = None
        self.last_status_wall = 0.0
        self.control_timer = self.create_timer(0.05, self.control_tick, clock=rclpy.clock.Clock())
        self.add_on_set_parameters_callback(lambda parameters: SetParametersResult(
            successful=False, reason='Restart the driver to change run configuration; use start/stop services for motion'))
        self.get_logger().info('PHASE 2: driver initially DISABLED; final commands still pass through SWAN')

    def transition(self, state, reason):
        if self.state != state or self.reason != reason:
            self.state_history.append({'sim': self.now_seconds(), 'state': state, 'reason': reason})
            self.state_history = self.state_history[-100:]
            self.get_logger().info(f'DRIVER {state}: {reason}')
        self.state, self.reason = state, reason

    def publish_command(self, speed=0.0, yaw_rate=0.0):
        command = Twist()
        command.linear.x, command.angular.z = float(speed), float(yaw_rate)
        self.cmd_pub.publish(command)
        self.last_command = (float(speed), float(yaw_rate))

    def stop(self, state, reason):
        self.archive_recovery('FAILED' if state == 'FAILED' else 'STOPPED', reason)
        self.transition(state, reason)
        self.paused = True
        self.invalidate(reason)
        self.follower.reset()
        self.publish_command()
        self.service_epoch += 1  # Late arming response cannot restart a stopped trial.
        self.need_off = True
        self.request_off()

    def archive_recovery(self, outcome, reason):
        if self.recovery is not None:
            record = self.recovery.summary(self.now_seconds())
            record.update({'outcome':outcome,'reason':reason})
            self.recovery_history.append(record)
            self.get_logger().info('RECOVERY_RESULT ' + json.dumps(record, allow_nan=False))
            self.recovery = None

    def request_off(self):
        now = time.monotonic()
        if not self.need_off or self.off_pending or now-self.last_off_request < 1.0:
            return
        self.last_off_request = now
        if not self.autonomy.service_is_ready():
            return
        request = SetBool.Request()
        request.data = False
        self.off_pending = True
        future = self.autonomy.call_async(request)
        epoch = self.service_epoch

        def done(result):
            self.off_pending = False
            try:
                response = result.result()
                if response.success and epoch == self.service_epoch:
                    self.need_off = False
            except Exception as error:
                self.get_logger().error(f'Emergency autonomy OFF failed: {error}')
        future.add_done_callback(done)

    def ready_to_start(self):
        now, wall = self.now_seconds(), time.monotonic()
        return (not self.fatal_error and not self.odom_fault and self.worker.ready.is_set()
                and self.frames.context(interval=self.value('context_interval_sec'),
                                        tolerance=self.value('context_tolerance_sec')) is not None
                and self.odom.poses and self.image_wall is not None and self.odom_wall is not None
                and 0 <= now-self.odom.poses[-1].stamp < self.value('sensor_timeout_sec')
                and 0 <= now-self.frames.frames[-1]['stamp'] < self.value('sensor_timeout_sec')
                and max(wall-self.image_wall, wall-self.odom_wall) < self.value('wall_sensor_timeout_sec'))

    def start_callback(self, request, response):
        if self.state not in TERMINAL or not self.ready_to_start() or self.off_pending or self.need_off:
            response.success = False
            response.message = 'Not ready: need fresh camera/odom, ready worker, stopped state and autonomy OFF acknowledgement'
            return response
        if not self.autonomy.service_is_ready():
            response.success, response.message = False, '/set_autonomy unavailable'
            return response
        if self.count_publishers('/cmd_vel_user') != 1:
            response.success, response.message = False, 'Stop other Teleop/AI publishers on /cmd_vel_user first'
            return response
        self.publish_command()
        self.follower.reset()
        self.invalidate('Start requested')
        self.route.reset()
        self.recovery, self.recovery_attempts = None, 0
        self.recovery_history = []
        self.arming_status = None
        self.service_epoch += 1
        epoch = self.service_epoch
        self.transition('ARMING', 'Waiting for SWAN mode acknowledgement')
        self.paused = True
        self.service_deadline = time.monotonic()+5.0
        mode = SetBool.Request()
        mode.data = self.value('assistance_enabled')
        future = self.autonomy.call_async(mode)

        def done(result):
            if epoch != self.service_epoch or self.state != 'ARMING':
                return
            try:
                ack = result.result()
                if not ack.success:
                    raise RuntimeError(ack.message)
                self.invalidate('SWAN mode acknowledged; fresh camera history required')
                self.paused = False
                self.run_started = self.now_seconds()
                self.progress_time = self.run_started
                self.progress_pose = self.odom.poses[-1] if self.odom.poses else None
                self.transition('WAITING', 'Gathering fresh six-frame context')
                if self.arming_status is not None:
                    message = String()
                    message.data = self.arming_status
                    self.assistance_callback(message)
            except Exception as error:
                self.stop('FAILED', f'autonomy_mode_failed: {error}')
        future.add_done_callback(done)
        response.success, response.message = True, 'Arming requested; inspect /ai_driver/driver_status'
        return response

    def stop_callback(self, request, response):
        self.stop('DISABLED', 'User stop')
        response.success, response.message = True, 'Zero sent; SWAN autonomy OFF requested'
        return response

    def estop_callback(self, request, response):
        self.stop('ESTOP', 'Emergency stop')
        response.success, response.message = True, 'Zero sent; SWAN autonomy OFF requested; check acknowledgement in status'
        return response

    def image_callback(self, message):
        before = self.image_count
        super().image_callback(message)
        if self.image_count > before:
            self.image_wall = time.monotonic()

    def odom_callback(self, message):
        before = self.odom_count
        super().odom_callback(message)
        if self.odom_count > before:
            self.actual_velocity = (message.twist.twist.linear.x, message.twist.twist.angular.z)
            if not all(math.isfinite(v) for v in self.actual_velocity):
                self.odom_fault='nonfinite_odom_velocity'
                self.actual_velocity=(0.0,0.0)
            pose = self.odom.poses[-1]
            if self.last_pose is not None:
                dt = pose.stamp-self.last_pose.stamp
                if dt > 0 and math.dist((pose.x, pose.y), (self.last_pose.x, self.last_pose.y)) > max(0.10, 2*dt):
                    self.odom_fault = 'odom_jump'
            self.last_pose = pose
            self.odom_wall = time.monotonic()
        else:
            self.odom_fault = 'invalid_or_out_of_order_odom'

    def invalidate(self, reason, clear_odom=False):
        super().invalidate(reason, clear_odom)
        if clear_odom and hasattr(self, 'state'):
            self.odom_fault = 'simulation_reset_requires_driver_restart'

    def handle_worker_result(self, result):
        previous = self.prediction_count
        super().handle_worker_result(result)
        if self.prediction_count > previous:
            self.latest_prediction_wall = time.monotonic()
            message = String()
            message.data = json.dumps(self.latest_result, allow_nan=False)
            self.prediction_pub.publish(message)

    def assistance_callback(self, message):
        status = message.data.strip().upper()
        self.assistance = status
        if not hasattr(self, 'state') or self.state in TERMINAL:
            return
        if self.state == 'ARMING':
            if status in ASSISTANCE_STATES or status in ('RECOVERED', 'COMPLETED'):
                self.arming_status = status
            return
        if not self.value('assistance_enabled'):
            if status == 'AUTONOMY_ON':
                self.stop('FAILED', 'SWAN mode changed externally')
            return  # Status never becomes a model input.
        if status == 'AUTONOMY_OFF':
            self.stop('FAILED', 'SWAN mode changed externally')
        elif status == 'BLOCKED':
            if self.recovery is not None:
                return
            if not self.value('use_sim_time'):
                self.stop('FAILED', 'blind_reverse_requires_simulation_time')
                return
            self.paused = True
            self.invalidate('BLOCKED; discard forward prediction')
            self.follower.reset()
            self.publish_command()
            # Zero removes only the per-run session cap; each ReverseRecovery
            # retains its own unchanged distance, time and odometry safeguards.
            limit = self.value('max_recovery_attempts')
            if not self.odom.poses or (limit > 0 and self.recovery_attempts >= limit):
                self.stop('FAILED', 'blocked_recovery_limit_or_missing_odom')
                return
            self.recovery_attempts += 1
            self.recovery = ReverseRecovery(self.odom.poses[-1], self.now_seconds(), time.monotonic())
            self.transition('SETTLING', 'BLOCKED reverse preparation')
        elif status in ('COMPLETED', 'RECOVERED', 'SLOPE_MANUAL', 'SLOPE_CLEAR', 'AUTONOMY_ON'):
            if self.recovery is not None and status != 'RECOVERED':
                return  # Never bypass the original BLOCKED release criteria.
            self.publish_command()  # RECOVERED preempts reverse immediately.
            self.archive_recovery('RECOVERED', 'Original SWAN released BLOCKED')
            self.follower.reset()
            self.publish_command()
            self.paused = True
            self.invalidate(f'{status}; discard old inference')
            self.rearm_until = self.now_seconds()+0.4
            self.transition('REARMING', f'{status}; zero input before fresh inference')
        elif status in ASSISTANCE_STATES:
            if self.recovery is not None:
                return
            if self.state != 'ASSISTING':
                self.assist_started = self.now_seconds()
                self.invalidate(f'SWAN owns control: {status}')
            self.paused = True
            self.follower.reset()
            self.publish_command()
            self.transition('ASSISTING', status)

    def control_tick(self):
        wall, now = time.monotonic(), self.now_seconds()
        dt = now-self.control_sim
        self.control_sim = now
        if self.state in TERMINAL:
            self.publish_command()
            self.request_off()
        elif self.state == 'ARMING':
            self.publish_command()
            if wall > self.service_deadline:
                self.stop('FAILED', 'autonomy_service_timeout')
        else:
            sensor_ok = (self.image_wall is not None and self.odom_wall is not None
                         and max(wall-self.image_wall, wall-self.odom_wall) <= self.value('wall_sensor_timeout_sec')
                         and self.frames.frames and self.odom.poses
                         and 0 <= now-self.frames.frames[-1]['stamp'] <= self.value('sensor_timeout_sec')
                         and 0 <= now-self.odom.poses[-1].stamp <= self.value('sensor_timeout_sec'))
            # Frames intentionally clear when SWAN takes over; use receipt age during that brief gap.
            if not self.frames.frames and self.image_wall is not None:
                sensor_ok = (self.odom.poses and self.odom_wall is not None
                             and max(wall-self.image_wall, wall-self.odom_wall) <= self.value('wall_sensor_timeout_sec')
                             and 0 <= now-self.odom.poses[-1].stamp <= self.value('sensor_timeout_sec'))
            if self.fatal_error or self.odom_fault or not sensor_ok or dt < 0 or dt > 0.3:
                self.stop('FAILED', self.fatal_error or self.odom_fault or 'sensor_or_clock_watchdog')
            elif now-self.run_started > self.value('max_run_sim_sec'):
                self.stop('FAILED', 'run_timeout')
            elif self.recovery is not None:
                phase, speed, reason = self.recovery.step(self.odom.poses[-1], now, wall)
                if phase == 'FAILED':
                    self.stop('FAILED', reason)
                else:
                    self.transition(phase, 'Bounded BLOCKED recovery')
                    self.publish_command(speed)
            elif self.state == 'ASSISTING':
                self.publish_command()
                if now-self.assist_started > 60.0:
                    self.stop('FAILED', 'assistance_timeout')
            elif self.state == 'REARMING':
                self.publish_command()
                if now >= self.rearm_until:
                    self.invalidate('Zero-input release complete; collect new frames')
                    self.paused = False
                    self.transition('WAITING', 'Fresh context after SWAN')
                    self.progress_time = now
                    self.progress_pose = self.odom.poses[-1]
            else:
                self.drive(now, wall, dt)
        if wall-self.last_status_wall > 0.2:
            self.publish_status()
            self.last_status_wall = wall

    def drive(self, now, wall, dt):
        pose = self.odom.poses[-1]
        _, reached = self.route.target(pose)
        if reached:
            self.stop('ARRIVED', 'Destination reached')
            return
        if self.progress_pose is not None and math.hypot(pose.x-self.progress_pose.x, pose.y-self.progress_pose.y) > 0.05:
            self.progress_pose, self.progress_time = pose, now
        if now-self.progress_time > 20.0:
            self.stop('FAILED', 'no_route_progress')
            return
        result = self.latest_result
        timeout = self.value('command_path_timeout_sec')
        if (result is None or self.latest_prediction_wall is None
                or not 0 <= now-result['image_stamp'] <= timeout
                or wall-self.latest_prediction_wall > 2.0):
            self.follower.reset()
            self.publish_command()
            self.transition('WAITING', 'Waiting for current prediction')
            return
        if dt <= 0:
            self.publish_command()  # Paused sim; do not keep a nonzero command queued.
            return
        remaining = math.dist((pose.x, pose.y), self.route.points[-1])
        try:
            speed, yaw, self.tracking = self.follower.command(result['path_odom_xy'], pose,
                                                             result['trajectory_xy_m'], remaining, min(dt, 0.2))
            self.publish_command(speed, yaw)
            self.transition('DRIVING', 'Following RGB-predicted path')
        except ValueError as error:
            self.stop('FAILED', str(error))

    def publish_status(self):
        message = String()
        pose = self.odom.poses[-1] if self.odom.poses else None
        message.data = json.dumps({
            'state': self.state, 'reason': self.reason, 'sim_time': self.now_seconds(),
            'assistance_enabled': self.value('assistance_enabled'), 'swan_status': self.assistance,
            'stop_autonomy_off_acknowledged': not self.need_off,
            'command': list(self.last_command), 'tracking': self.tracking,
            'pose': vars(pose) if pose else None, 'predictions': self.prediction_count,
            'actual_velocity': list(self.actual_velocity),
            'route_index': self.route.index, 'route_points_odom': self.route.points,
            'waypoint_odom_xy': self.route.points[self.route.index],
            'final_goal_odom_xy': self.route.points[-1],
            'distance_to_goal_m': math.dist((pose.x, pose.y), self.route.points[-1]) if pose else None,
            'temporary_goal_odom_xy': self.latest_result.get('target_odom_xy') if self.latest_result else None,
            'last_prediction_image_stamp': self.latest_result.get('image_stamp') if self.latest_result else None,
            'recovery_attempts': self.recovery_attempts, 'history': self.state_history,
            'recovery': self.recovery.summary(self.now_seconds()) if self.recovery else None,
            'recovery_history': self.recovery_history,
        }, allow_nan=False)
        self.state_pub.publish(message)

    def destroy_node(self):
        if rclpy.ok():
            self.publish_command()
            self.need_off = True
            self.last_off_request = -math.inf
            self.request_off()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MbraDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        import signal
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
