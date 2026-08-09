#!/usr/bin/env python3
"""
[인지 노드 - LiDAR]  /scan(LaserScan) → /swan/detections
GT 스텁을 대체하는 '진짜' 인지. 거리·각도만 채움 (클래스는 카메라가 나중에).

처리:
  1) 유효범위 필터 (min~max, inf/nan 제거)
  2) 전방 콘(±fov) 만 관심
  3) 클러스터링: 인접 점(각도·거리 근접) = 하나의 장애물
  4) 각 클러스터 → 최근접 거리(표면) + 중심 각도
     ※ assist_node 와 규약 동일: distance = 표면까지, angle = +좌/-우

★ 이 노드만 붙이면 뒷단(assist/control)이 그대로 회피 시작.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from swan_interfaces.msg import Detection, DetectionArray


def norm_angle(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class LidarPerception(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('out_topic', '/swan/detections')
        self.declare_parameter('fov', 0.9)              # 전방 ±rad (앞쪽만 관심)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('cluster_gap', 0.30)     # 이웃 점 거리차 > 이면 다른 물체
        self.declare_parameter('min_points', 2)         # 노이즈 제거: 최소 점 수
        self.declare_parameter('robot_front', 0.41)     # LiDAR→앞범퍼 보정(표면 clearance)
        g = lambda n: self.get_parameter(n).value
        self.fov, self.max_range = g('fov'), g('max_range')
        self.gap, self.min_pts = g('cluster_gap'), g('min_points')
        self.front = g('robot_front')

        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, 10)
        self.pub = self.create_publisher(DetectionArray, g('out_topic'), 10)
        self.get_logger().info('perception(LiDAR) 시작: /scan → 클러스터링 → detections')

    def on_scan(self, scan: LaserScan):
        # 각도별 (angle, range) 중 전방·유효만
        pts = []
        a = scan.angle_min
        for r in scan.ranges:
            ang = norm_angle(a)
            if (scan.range_min < r < min(self.max_range, scan.range_max)
                    and not math.isinf(r) and not math.isnan(r)
                    and abs(ang) < self.fov):
                pts.append((ang, r))
            a += scan.angle_increment

        # 각도 순 정렬 후 클러스터링 (거리 급변 = 경계)
        pts.sort()
        clusters, cur = [], []
        for i, (ang, r) in enumerate(pts):
            if cur and abs(r - cur[-1][1]) > self.gap:
                clusters.append(cur); cur = []
            cur.append((ang, r))
        if cur:
            clusters.append(cur)

        out = DetectionArray()
        out.header = scan.header
        out.img_w = out.img_h = 0
        for cl in clusters:
            if len(cl) < self.min_pts:
                continue
            # 최근접 거리(표면) + 그 점 근처 각도(가장 가까운 점의 각도)
            near = min(cl, key=lambda p: p[1])
            surface = max(0.0, near[1] - self.front)   # 표면까지 clearance 규약
            d = Detection()
            d.class_id, d.class_name = 99, 'unknown'    # LiDAR는 클래스 모름
            d.confidence = 1.0
            d.distance_m = float(surface)
            d.angle_rad = float(near[0])                # +좌/-우
            d.risk_level = 2 if surface < 1.5 else 1
            out.detections.append(d)

        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(LidarPerception())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
