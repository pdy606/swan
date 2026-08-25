#!/usr/bin/env python3
"""
[퓨전 노드 — 자립형 퍼셉션 (통합 코어)]
lidar_perception(클러스터링) + yolo_adapter(클래스 결합)를 **하위로 흡수**한 단일 퍼셉션 노드.
팀 입력을 직접 받아 최종 인지 결과 하나만 낸다.

입력:
  /scan_filtered           (규원, LaserScan)  — 자기몸체 제거된 스캔
  /yolo/detected_objects   (다영, String)     — "Detected: a, b" 라벨
출력:
  /swan/detections         (DetectionArray)   — 거리=LiDAR, 클래스=YOLO

동작:
  1) 스캔 → 전방 클러스터링 → 물체별 (거리, 각도)
  2) YOLO 라벨을 안전 우선순위(person 우선)로 가까운 물체에 배정 (다영 출력엔 위치 없어 각도 차용)
  3) 결합해서 /swan/detections 발행
※ 신호등 등 '의미기반' 신호 클래스는 여기서 제외(물리 장애물 아님) → signal_node 가 담당.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from swan_interfaces.msg import Detection, DetectionArray

# 다영 YOLO 클래스 → 내부 (class_id, name)
NAME2OURS = {
    'person': (3, 'person'),
    'astrid': (3, 'person'), 'delivery': (3, 'person'), 'rena': (3, 'person'),
    'rusty': (3, 'person'), 'stacy': (3, 'person'), 'trish': (3, 'person'), 'wendy': (3, 'person'),
    'car': (1, 'car'), 'bus': (1, 'car'), 'truck': (1, 'car'),
    'bicycle': (0, 'kickboard'), 'motorcycle': (0, 'kickboard'), 'kickboard': (0, 'kickboard'),
}
PRIORITY = ['person', 'car', 'kickboard']
# 신호등(의미기반) — 다이어그램 ⑤ 처럼 LiDAR 거리와 결합해 "정면 Om 빨간불" 로 출력
SIGNAL_CLASSES = {
    'traffic light red': (12, 'traffic_light_red'),
    'traffic light green': (12, 'traffic_light_green'),
}
# 통과 가능(주행면) — 물체 배정 제외
TRAVERSABLE = {'faded_crosswalk', 'intact_crosswalk', 'blind tracks', 'curb ramp',
               'zebra crossing'}


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('scan_topic', '/scan_filtered')
        self.declare_parameter('yolo_topic', '/yolo/detected_objects')
        self.declare_parameter('out_topic', '/swan/detections')
        self.declare_parameter('fov', 0.9)              # 전방 ±rad
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('cluster_gap', 0.30)     # 이웃 점 거리차 > 이면 다른 물체
        self.declare_parameter('min_points', 2)
        self.declare_parameter('robot_front', 0.41)     # LiDAR→앞범퍼 보정
        self.declare_parameter('yolo_timeout', 1.5)
        g = lambda n: self.get_parameter(n).value
        self.fov, self.max_r = g('fov'), g('max_range')
        self.gap, self.min_pts, self.front = g('cluster_gap'), g('min_points'), g('robot_front')
        self.yolo_to = g('yolo_timeout')

        self.yolo = []; self.yolo_stamp = None
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, 10)
        self.create_subscription(String, g('yolo_topic'), self.on_yolo, 10)
        self.pub = self.create_publisher(DetectionArray, g('out_topic'), 10)
        self.get_logger().info('fusion(자립형): /scan_filtered + /yolo → /swan/detections')

    def on_yolo(self, m: String):
        txt = m.data.split(':', 1)[-1] if ':' in m.data else m.data
        self.yolo = [x.strip().lower() for x in txt.split(',') if x.strip()]
        self.yolo_stamp = self.get_clock().now()

    def _yolo_fresh(self):
        return self.yolo_stamp is not None and \
            (self.get_clock().now() - self.yolo_stamp).nanoseconds / 1e9 < self.yolo_to

    # ---- 하위 흡수 ①: LiDAR 전방 클러스터링 → 물체 목록 ----
    def cluster(self, scan: LaserScan):
        pts = []      # (거리, 각도)
        a = scan.angle_min
        for r in scan.ranges:
            if abs(a) <= self.fov and scan.range_min < r < self.max_r:
                pts.append((r, a))
            a += scan.angle_increment
        pts.sort(key=lambda p: p[1])            # 각도순
        clusters, cur = [], []
        for p in pts:
            if cur and abs(p[0] - cur[-1][0]) > self.gap:
                if len(cur) >= self.min_pts:
                    clusters.append(cur)
                cur = []
            cur.append(p)
        if len(cur) >= self.min_pts:
            clusters.append(cur)
        obs = []
        for cl in clusters:
            near = min(cl, key=lambda p: p[0])
            surface = max(0.0, near[0] - self.front)     # 표면 clearance
            obs.append((surface, near[1]))               # (거리, 각도)
        obs.sort(key=lambda o: o[0])                     # 가까운 순
        return obs

    # ---- 하위 흡수 ②: YOLO 라벨을 우선순위로 물체에 배정 ----
    def assign(self, obs):
        cls = []
        if self._yolo_fresh():
            cand = [c for c in self.yolo if c in NAME2OURS]
            cls = sorted(set(cand), key=lambda c: PRIORITY.index(NAME2OURS[c][1])
                         if NAME2OURS[c][1] in PRIORITY else 99)
        out = DetectionArray()
        for i, (dist, ang) in enumerate(obs):
            d = Detection()
            d.distance_m, d.angle_rad = float(dist), float(ang)
            d.risk_level = 2 if dist < 1.5 else 1
            if i < len(cls):
                d.class_id, d.class_name = NAME2OURS[cls[i]]
                d.confidence = 0.5
            else:
                d.class_id, d.class_name, d.confidence = 99, 'unknown', 1.0
            out.detections.append(d)

        # ── 신호등 결합 (다이어그램 ⑤): 카메라 신호 + LiDAR 거리 → "정면 Om 신호등" ──
        if self._yolo_fresh():
            sig = next((c for c in self.yolo if c in SIGNAL_CLASSES), None)
            if sig:
                cid, cname = SIGNAL_CLASSES[sig]
                # 화면 정면 신호 → 전방(각도 최소) LiDAR 구조물의 거리를 차용
                front = min(obs, key=lambda o: abs(o[1])) if obs else (0.0, 0.0)
                d = Detection()
                d.class_id, d.class_name, d.confidence = cid, cname, 0.9
                d.distance_m, d.angle_rad = float(front[0]), float(front[1])
                d.risk_level = 2 if 'red' in cname else 0
                out.detections.append(d)     # 예: traffic_light_red @ 6.0m
        return out

    def on_scan(self, scan: LaserScan):
        obs = self.cluster(scan)
        out = self.assign(obs)
        out.header = scan.header
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
