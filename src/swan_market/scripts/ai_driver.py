#!/usr/bin/env python3
"""Experimental Qwen operator: LiDAR + odometry -> /cmd_vel_user.

Run only in Gazebo with the wheelchair_navigation driving supervisor. This
script never publishes to /model/wheelchair/cmd_vel directly.
"""

import argparse
import csv
import math
from pathlib import Path
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from ai_driver_core import ACTIONS, bounded_action, normalize, ollama_action, scan_clearances


class AIDriver(Node):
    def __init__(self, args):
        super().__init__('ai_market_driver')
        self.args = args
        self.publisher = self.create_publisher(Twist, '/cmd_vel_user', 10)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.scan = None
        self.odom = None
        self.scan_received = 0.0
        self.odom_received = 0.0
        self.current_action = 'stop'
        self.action_until = 0.0
        self.in_flight = False
        self.next_request = 0.0
        self.started = time.monotonic()
        self.finished = False
        self.finish_reason = ''
        self.lock = threading.Lock()
        self.args.log.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.args.log.open('w', newline='')
        self.rows = csv.writer(self.log)
        self.rows.writerow(['wall_elapsed_s', 'sim_time_s', 'x', 'y', 'yaw',
                            'goal_distance_m', 'heading_error_rad', 'front_m',
                            'left_m', 'right_m', 'proposed', 'applied',
                            'gate', 'api_latency_s', 'reason'])
        self.create_timer(0.1, self.publish_command)
        self.create_timer(0.2, self.maybe_request)

    def on_scan(self, message):
        self.scan = message
        self.scan_received = time.monotonic()

    def on_odom(self, message):
        self.odom = message
        self.odom_received = time.monotonic()

    def observation(self):
        if self.scan is None or self.odom is None:
            return None
        pose = self.odom.pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        dx = self.args.goal_x - pose.position.x
        dy = self.args.goal_y - pose.position.y
        clearance = scan_clearances(self.scan.ranges, self.scan.angle_min,
                                    self.scan.angle_increment, self.scan.range_max)
        distance = math.hypot(dx, dy)
        heading = normalize(math.atan2(dy, dx) - yaw)
        stamp = self.odom.header.stamp
        state = {
            'goal_distance_m': round(distance, 3),
            'heading_error_rad': round(heading, 3),
            'front_clearance_m': round(clearance['front'], 3),
            'left_clearance_m': round(clearance['left'], 3),
            'right_clearance_m': round(clearance['right'], 3),
        }
        row = [round(time.monotonic() - self.started, 3),
               round(stamp.sec + stamp.nanosec / 1e9, 3),
               round(pose.position.x, 3), round(pose.position.y, 3),
               round(yaw, 3), round(distance, 3), round(heading, 3),
               round(clearance['front'], 3), round(clearance['left'], 3),
               round(clearance['right'], 3)]
        return state, clearance, distance, row

    def publish_command(self):
        now = time.monotonic()
        with self.lock:
            action = self.current_action if now < self.action_until else 'stop'
        if now - self.scan_received > 2.0 or now - self.odom_received > 2.0:
            action = 'stop'
        elif self.scan is not None and self.odom is not None:
            clearance = scan_clearances(self.scan.ranges, self.scan.angle_min,
                                        self.scan.angle_increment, self.scan.range_max)
            pose = self.odom.pose.pose.position
            distance = math.hypot(self.args.goal_x - pose.x, self.args.goal_y - pose.y)
            action, _ = bounded_action(action, clearance, distance)
        command = Twist()
        command.linear.x, command.angular.z = ACTIONS[action]
        self.publisher.publish(command)
        if now - self.started >= self.args.max_wall_seconds:
            self.finished = True
            self.finish_reason = 'wall_timeout'

    def maybe_request(self):
        now = time.monotonic()
        if self.finished or self.in_flight or now < self.next_request:
            return
        if now - self.scan_received > 2.0 or now - self.odom_received > 2.0:
            return
        snapshot = self.observation()
        if snapshot is None:
            return
        state, clearance, distance, row = snapshot
        if distance <= 0.30:
            self.finished = True
            self.finish_reason = 'goal_reached'
            self.get_logger().info(f'Goal reached at x={row[2]:.3f} y={row[3]:.3f}; '
                                   f'distance={distance:.3f}m')
            return
        self.in_flight = True
        threading.Thread(target=self.ask_model,
                         args=(state, clearance, distance, row), daemon=True).start()

    def ask_model(self, state, clearance, distance, row):
        start = time.monotonic()
        proposed, reason, gate = 'stop', '', 'api_error'
        try:
            proposed, reason, _ = ollama_action(self.args.endpoint, self.args.model,
                                                state, timeout=self.args.api_timeout)
            applied, gate = bounded_action(proposed, clearance, distance)
        except Exception as error:
            applied = 'stop'
            reason = f'{type(error).__name__}: {error}'[:160]
        latency = time.monotonic() - start
        with self.lock:
            if self.finished:
                self.in_flight = False
                return
            self.current_action = applied
            self.action_until = time.monotonic() + self.args.command_seconds
            self.next_request = time.monotonic() + self.args.request_interval
            self.in_flight = False
            self.rows.writerow(row + [proposed, applied, gate, round(latency, 3), reason])
            self.log.flush()
        self.get_logger().info(f'AI {proposed} -> {applied} ({gate}), {latency:.2f}s; '
                               f'distance={distance:.2f}m front={clearance["front"]:.2f}m')

    def close(self):
        self.finished = True
        for _ in range(5):
            self.publisher.publish(Twist())
            time.sleep(0.1)
        snapshot = self.observation()
        if snapshot is not None:
            _, _, _, row = snapshot
            with self.lock:
                self.rows.writerow(row + ['stop', 'stop', self.finish_reason or 'closed',
                                          0.0, 'final observation'])
                self.log.flush()
            self.get_logger().info(f'Final pose x={row[2]:.3f} y={row[3]:.3f}; '
                                   f'distance={row[5]:.3f}m; '
                                   f'reason={self.finish_reason or "closed"}')
        self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint', default='http://127.0.0.1:11434/api/chat')
    parser.add_argument('--model', default='qwen3:8b')
    parser.add_argument('--scan-topic', default='/scan_filtered')
    parser.add_argument('--goal-x', type=float, required=True)
    parser.add_argument('--goal-y', type=float, required=True)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--max-wall-seconds', type=float, default=90)
    parser.add_argument('--command-seconds', type=float, default=1.5)
    parser.add_argument('--request-interval', type=float, default=0.1)
    parser.add_argument('--api-timeout', type=float, default=10)
    args = parser.parse_args()
    rclpy.init()
    node = AIDriver(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
