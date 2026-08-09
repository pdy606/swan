#!/usr/bin/env python3
"""
[인지 노드 - Ground Truth 버전]  ★ 나중에 LiDAR 로 교체 ★
LiDAR/카메라 대신 '장애물의 실제 위치(Gazebo가 아는 값)'로 거리·각도를 계산해 발행.
→ 인지 품질과 무관하게 '제어가 진짜 박스를 피하는지' 먼저 검증하기 위한 대역.

★ 거리 규약: LiDAR 와 동일하게 **표면까지의 여유거리(clearance)** 를 보고한다.
   distance_m = 중심간거리 - 로봇앞범퍼오프셋 - 장애물반경
   (중심↔중심으로 재면 전장 0.82m 휠체어에선 '0.8m'가 이미 충돌인 함정이 생김)
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from swan_interfaces.msg import Detection, DetectionArray


def yaw_from_quat(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def norm_angle(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class GtPerception(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.declare_parameter('obs_x', 4.0)
        self.declare_parameter('obs_y', 0.4)
        self.declare_parameter('obs_radius', 0.25)     # 박스 반폭
        self.declare_parameter('robot_front', 0.41)    # base_link → 앞범퍼 (전장 0.82/2)
        self.declare_parameter('fov', 0.9)             # 시야각 ±rad
        self.declare_parameter('max_range', 8.0)
        # 테스트용 클래스 지정 (카메라가 나중에 대체): 0:kickboard 1:car 3:person ...
        self.declare_parameter('obs_class_id', 0)
        self.declare_parameter('obs_class_name', 'kickboard')
        g = lambda n: self.get_parameter(n).value
        self.ox, self.oy = g('obs_x'), g('obs_y')
        self.orad, self.front = g('obs_radius'), g('robot_front')
        self.fov, self.max_range = g('fov'), g('max_range')
        self.cls_id, self.cls_name = g('obs_class_id'), g('obs_class_name')

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(DetectionArray, '/swan/detections', 10)
        self.robot = (0.0, 0.0, 0.0)
        self.create_timer(0.05, self.tick)  # 20 Hz (반응지연 축소)
        self.get_logger().info('perception(GT) 시작: 표면까지 여유거리(clearance) 발행')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot = (p.x, p.y, yaw_from_quat(q.z, q.w))

    def tick(self):
        rx, ry, ryaw = self.robot
        dx, dy = self.ox - rx, self.oy - ry
        center_d = math.hypot(dx, dy)
        clearance = center_d - self.orad - self.front   # LiDAR 처럼 표면까지
        bearing = norm_angle(math.atan2(dy, dx) - ryaw)

        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.img_w, msg.img_h = 0, 0

        if center_d < self.max_range and abs(bearing) < self.fov:
            d = Detection()
            d.class_id, d.class_name = self.cls_id, self.cls_name
            d.confidence = 0.95
            d.distance_m = float(max(0.0, clearance))
            d.angle_rad = float(bearing)
            d.risk_level = 2 if clearance < 1.5 else 1
            msg.detections.append(d)

        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(GtPerception())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
