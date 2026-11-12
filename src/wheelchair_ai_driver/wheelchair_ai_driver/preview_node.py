"""ROS-only MBRA trajectory preview. This executable has NO actuation publisher."""
import base64
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .inference_client import InferenceClient
from .observation import FrameBuffer, OdomBuffer, Pose2D, quaternion_yaw
from .waypoints import RouteTracker, relative_goal, world_to_odom

ASSISTANCE_STATES = {'STOPPED', 'WAITING_FOR_NAV', 'AVOIDING', 'REPLANNING',
                     'VERIFYING_CLEAR', 'BLOCKED'}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class InferencePreview(Node):
    def __init__(self, node_name='mbra_inference_preview', extra_defaults=None):
        super().__init__(node_name)
        self.preview_only = node_name == 'mbra_inference_preview'
        defaults = {
            'package_root': str(Path(__file__).resolve().parents[1]),
            'camera_topic': '/camera', 'odom_topic': '/odom',
            'inference_python': '', 'model_path': '', 'provider': 'cpu', 'threads': 2,
            'context_interval_sec': 0.2, 'context_tolerance_sec': 0.075,
            'sensor_timeout_sec': 0.5, 'odom_sync_tolerance_sec': 0.15,
            'result_timeout_sec': 1.0, 'worker_timeout_sec': 30.0,
            'lookahead_m': 3.0, 'waypoint_radius_m': 0.8, 'goal_radius_m': 0.5,
            'world_to_odom_translation': [0.0, 0.0], 'world_to_odom_yaw': 0.0,
            'route_world_xy': [-35.0, 0.0, 20.5, 0.0, 30.0, 2.2, 37.5, 0.0,
                               45.0, -2.2, 53.0, 0.0],
            'result_file': 'runtime/reports/latest_preview.json',
            'log_period_sec': 2.0,
        }
        defaults.update(extra_defaults or {})
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.root = Path(self.value('package_root')).expanduser().resolve()
        python_path = self.value('inference_python') or self.root/'.venv_mbra/bin/python'
        model_path = self.value('model_path') or self.root/'assets/navigation_model_zoo/MBRA_PG_Official/mbra.onnx'
        route = self.value('route_world_xy')
        translation = self.value('world_to_odom_translation')
        if len(route) < 2 or len(route) % 2 or len(translation) != 2:
            raise ValueError('route_world_xy must contain XY pairs; translation must have two values')
        if not all(math.isfinite(value) for value in [*route, *translation, self.value('world_to_odom_yaw')]):
            raise ValueError('Route and coordinate transform must be finite')
        for name in ('context_interval_sec', 'context_tolerance_sec', 'sensor_timeout_sec',
                     'odom_sync_tolerance_sec', 'result_timeout_sec', 'worker_timeout_sec'):
            if not math.isfinite(self.value(name)) or self.value(name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.value('threads') < 1:
            raise ValueError('threads must be positive')
        points = [world_to_odom(route[index:index+2], translation, self.value('world_to_odom_yaw'))
                  for index in range(0, len(route), 2)]
        self.route = RouteTracker(points, self.value('waypoint_radius_m'),
                                  self.value('goal_radius_m'), self.value('lookahead_m'))
        self.result_file = None
        if self.value('result_file'):
            self.result_file = (self.root/self.value('result_file')).resolve()
            if not self.result_file.is_relative_to(self.root):
                raise ValueError('result_file must stay inside wheelchair_ai_driver')
            self.result_file.parent.mkdir(parents=True, exist_ok=True)
        self.frames = FrameBuffer()
        self.odom = OdomBuffer()
        self.odom_frame = None
        self.base_frame = None
        self.camera_frame = None
        self.last_frame_received = None
        self.last_odom_received = None
        self.last_clock = None
        self.last_submitted_stamp = -1.0
        self.generation = 0
        self.sequence = 0
        self.pending = None
        self.last_log = 0.0
        self.assistance = 'UNKNOWN'
        self.paused = False
        self.fatal_error = None
        self.latest_result = None
        self.prediction_count = 0
        self.image_count = 0
        self.odom_count = 0
        self.dropped_results = 0
        self.path_pub = self.create_publisher(RosPath, '/ai_driver/predicted_path', 10)
        self.diagnostic_pub = self.create_publisher(String, '/ai_driver/inference_status', 10)
        self.image_sub = self.create_subscription(Image, self.value('camera_topic'),
                                                  self.image_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, self.value('odom_topic'),
                                                 self.odom_callback, qos_profile_sensor_data)
        self.assistance_sub = self.create_subscription(String, '/avoidance_status',
                                                       self.assistance_callback, 10)
        self.worker = InferenceClient(python_path, model_path, self.value('provider'),
                                      self.value('threads'), self.value('worker_timeout_sec'))
        self.timer = self.create_timer(0.05, self.tick)
        # Wall-clock reporting remains alive if Gazebo/clock is absent or paused.
        self.diagnostic_timer = self.create_timer(2.0, self.publish_diagnostic,
                                                   clock=rclpy.clock.Clock())
        if self.preview_only:
            self.get_logger().info('PHASE 1: trajectory preview only; no velocity publisher or autonomy service client')
        self.get_logger().info('Route uses configured T_odom_world (identity by default); verify the initial odom pose')

    def value(self, name):
        return self.get_parameter(name).value

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_clock(self):
        now = self.now_seconds()
        if self.last_clock is not None and now < self.last_clock - 1e-6:
            self.invalidate('Simulation clock moved backwards', clear_odom=True)
        self.last_clock = now

    def invalidate(self, reason, clear_odom=False):
        self.generation += 1
        self.frames.clear()
        self.last_submitted_stamp = -1.0
        self.latest_result = None
        self.pending = None
        if clear_odom:
            self.odom.clear()
            self.last_frame_received = None
            self.last_odom_received = None
            self.route.reset()
        self.get_logger().info(f'Preview context reset: {reason}')

    def image_callback(self, message):
        self.check_clock()
        if message.encoding.lower() not in ('rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8'):
            self.get_logger().warning(f'Unsupported image encoding: {message.encoding}', throttle_duration_sec=5)
            return
        stamp = stamp_seconds(message.header.stamp)
        frame = {'stamp': stamp, 'width': message.width, 'height': message.height,
                 'step': message.step, 'encoding': message.encoding,
                 'data': base64.b64encode(bytes(message.data)).decode('ascii')}
        try:
            if self.frames.append(frame):
                self.image_count += 1
                self.last_frame_received = self.now_seconds()
                self.camera_frame = message.header.frame_id
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5)

    def odom_callback(self, message):
        self.check_clock()
        if not message.header.frame_id or not message.child_frame_id:
            self.get_logger().warning('Odometry must identify parent and child frames', throttle_duration_sec=5)
            return
        if self.odom_frame is not None and (message.header.frame_id != self.odom_frame
                                           or message.child_frame_id != self.base_frame):
            self.fatal_error = 'Odometry frame IDs changed during the preview'
            return
        orientation = message.pose.pose.orientation
        position = message.pose.pose.position
        try:
            pose = Pose2D(stamp_seconds(message.header.stamp), position.x, position.y,
                          quaternion_yaw(orientation.x, orientation.y, orientation.z, orientation.w))
            if self.odom.append(pose):
                self.odom_count += 1
                self.last_odom_received = self.now_seconds()
                self.odom_frame = message.header.frame_id
                self.base_frame = message.child_frame_id
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5)

    def assistance_callback(self, message):
        # Status is never passed into the MBRA worker. No recovery is implemented in phase 1.
        status = message.data.strip().upper()
        self.assistance = status
        if status in ASSISTANCE_STATES:
            if not self.paused:
                self.invalidate(f'assistance {status}')
            self.paused = True
        elif status in ('COMPLETED', 'RECOVERED', 'AUTONOMY_OFF', 'AUTONOMY_ON',
                        'SLOPE_MANUAL', 'SLOPE_CLEAR'):
            self.paused = False
            self.invalidate(f'assistance {status}; require a fresh six-frame context')

    def tick(self):
        self.check_clock()
        for result in self.worker.poll():
            self.handle_worker_result(result)
        if self.fatal_error or self.paused or self.worker.busy or not self.worker.ready.is_set():
            return
        context = self.frames.context(interval=self.value('context_interval_sec'),
                                      tolerance=self.value('context_tolerance_sec'))
        if context is None:
            return
        stamp = context[-1]['stamp']
        # Camera 10 Hz -> inference at most 5 Hz, using simulated timestamps.
        if stamp < self.last_submitted_stamp + self.value('context_interval_sec') - 1e-6:
            return
        now = self.now_seconds()
        age = now - stamp
        if age < -0.05 or age > self.value('sensor_timeout_sec'):
            return
        if self.last_odom_received is None or now - self.last_odom_received > self.value('sensor_timeout_sec'):
            return
        pose = self.odom.pose_at(stamp, self.value('odom_sync_tolerance_sec'))
        if pose is None:
            return
        target, reached = self.route.target(pose)
        if reached:
            return
        goal = relative_goal(target, pose)
        self.sequence += 1
        request = {'type': 'infer', 'id': self.sequence, 'generation': self.generation,
                   'frames': context, 'goal_xy': goal}
        if self.worker.submit(request):
            self.last_submitted_stamp = stamp
            self.pending = {'id': self.sequence, 'generation': self.generation, 'stamp': stamp,
                            'frame_stamps': [frame['stamp'] for frame in context],
                            'pose': pose, 'goal_xy': goal, 'target_odom_xy': target,
                            'submitted_wall': time.monotonic()}

    def handle_worker_result(self, result):
        kind = result.get('type')
        if kind == 'ready':
            self.get_logger().info(f'MBRA worker ready: {json.dumps(result["model"])}')
            return
        if kind == 'fatal':
            self.fatal_error = result['error']
            self.get_logger().error(f'MBRA worker failed: {self.fatal_error}')
            return
        pending = self.pending
        if (pending is None or result.get('id') != pending['id']
                or result.get('generation') != self.generation):
            self.dropped_results += 1
            return
        self.pending = None
        if kind == 'error':
            self.get_logger().error(f'Inference rejected: {result["error"]}')
            return
        age = self.now_seconds() - pending['stamp']
        wall_age = time.monotonic() - pending['submitted_wall']
        if self.paused or age < -0.05 or max(age, wall_age) > self.value('result_timeout_sec'):
            self.dropped_results += 1
            return
        pose = pending['pose']
        trajectory = result['trajectory_xy_m']
        if len(trajectory) != 8 or any(len(point) != 2 or not all(math.isfinite(v) for v in point)
                                       for point in trajectory):
            self.fatal_error = 'Invalid trajectory response'
            return
        path = RosPath()
        path.header.frame_id = self.odom_frame
        stamp_ns = round(pending['stamp'] * 1e9)
        path.header.stamp.sec, path.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        for index, (x, y) in enumerate(trajectory):
            waypoint = PoseStamped()
            waypoint.header = path.header
            waypoint.pose.position.x = pose.x + cosine*x - sine*y
            waypoint.pose.position.y = pose.y + sine*x + cosine*y
            # Upstream raw waypoint order is x, y, cos(yaw), sin(yaw).
            raw = result['raw_waypoints'][index]
            yaw = pose.yaw + math.atan2(raw[3], raw[2])
            waypoint.pose.orientation.z = math.sin(yaw / 2)
            waypoint.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(waypoint)
        self.path_pub.publish(path)
        self.prediction_count += 1
        self.latest_result = {
            'dry_run_only': self.preview_only, 'camera_topic': self.value('camera_topic'),
            'odom_topic': self.value('odom_topic'), 'odom_frame': self.odom_frame,
            'base_frame': self.base_frame, 'camera_frame': self.camera_frame,
            'frame_stamps': pending['frame_stamps'], 'image_stamp': pending['stamp'],
            'pose_at_image': {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw},
            'goal_xy': pending['goal_xy'], 'target_odom_xy': pending['target_odom_xy'],
            'route_index': self.route.index, 'prediction_count': self.prediction_count,
            'result_age_sim_sec': age, 'round_trip_wall_ms': wall_age * 1000,
            'path_odom_xy': [[point.pose.position.x, point.pose.position.y] for point in path.poses],
            **result,
        }
        if self.result_file:
            temporary = self.result_file.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.latest_result, indent=2, allow_nan=False) + '\n')
            temporary.replace(self.result_file)
        if time.monotonic() - self.last_log >= self.value('log_period_sec'):
            self.get_logger().info(f'{"PREVIEW ONLY" if self.preview_only else "PREDICTION"} goal={pending["goal_xy"]} '
                                   f'waypoints={trajectory} inference={result["total_ms"]:.1f} ms')
            self.last_log = time.monotonic()

    def publish_diagnostic(self):
        now = self.now_seconds()
        message = String()
        message.data = json.dumps({
            'dry_run_only': self.preview_only, 'worker_ready': self.worker.ready.is_set(),
            'images_received': self.image_count, 'odom_received': self.odom_count,
            'predictions': self.prediction_count, 'dropped_results': self.dropped_results,
            'camera_topic': self.value('camera_topic'), 'odom_topic': self.value('odom_topic'),
            'last_image_age_sim_sec': now-self.frames.frames[-1]['stamp'] if self.frames.frames else None,
            'last_odom_age_sim_sec': now-self.odom.poses[-1].stamp if self.odom.poses else None,
            'assistance_status': self.assistance, 'preview_paused': self.paused,
            'fatal_error': self.fatal_error,
        }, allow_nan=False)
        self.diagnostic_pub.publish(message)

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InferencePreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
