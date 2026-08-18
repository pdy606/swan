#!/usr/bin/env python3
"""
[YOLO 어댑터]  dayoung 팀원 YOLO 출력 → 내 fusion 계약으로 변환.

문제: dayoung `/yolo/detected_objects`(std_msgs/String)는 **클래스 이름만** 발행한다.
      예) "Detected: person, car"  ← bbox·좌표·개수·신뢰도·신호등색 전혀 없음.
      반면 내 fusion 은 **각도(angle_rad)로** 카메라·LiDAR 를 매칭한다.

해결: 위치가 없으니 **LiDAR 최근접 물체의 각도를 빌려** 클래스에 공간정보를 부여한다.
      - YOLO 가 본 클래스들을 안전 우선순위(person>car>...)로 정렬
      - LiDAR 물체를 가까운 순으로 정렬해 1:1 로 배정
      → 각 클래스가 LiDAR 물체의 각도/거리를 갖게 됨
      → 그 결과를 /swan/cam_detections 로 발행하면 **기존 fusion 이 그대로 각도매칭**한다.
        (fusion 코드 수정 불필요)

입력:  /yolo/detected_objects (String)         ← dayoung
       /swan/lidar_detections (DetectionArray)  ← 내 lidar_perception (각도·거리)
출력:  /swan/cam_detections   (DetectionArray)  → 내 fusion

⚠️ 한계 (dayoung 출력 형식상 불가피):
  1. 위치 없음 → '가까운 물체 = YOLO가 본 것' 가정. 물체 여러 개면 배정이 근사적.
  2. 신호등 빨강/초록 구분 불가(문자열에 색 없음) → 신호 정지는 이 경로로 못 함.
  → 정밀히 하려면 dayoung YOLO 가 bbox 를 함께 내보내거나,
    내 camera_perception(detector:=yolo)로 /camera 를 직접 처리하는 편이 낫다.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swan_interfaces.msg import Detection, DetectionArray

# YOLO 클래스 이름(소문자) → 내 내부 (class_id, name).  COCO/커스텀 공통 근사.
NAME2OURS = {
    'person': (3, 'person'),
    'car': (1, 'car'), 'bus': (1, 'car'), 'truck': (1, 'car'),
    'bicycle': (0, 'kickboard'), 'motorcycle': (0, 'kickboard'), 'kickboard': (0, 'kickboard'),
    'curb': (2, 'curb'), 'pole': (5, 'pole'),
}
# 안전 우선순위: 사람(양보)이 가장 중요 → 가까운 물체에 먼저 배정
PRIORITY = ['person', 'car', 'kickboard', 'curb', 'pole']


class YoloAdapter(Node):
    def __init__(self):
        super().__init__('yolo_adapter_node')
        self.declare_parameter('yolo_topic', '/yolo/detected_objects')
        self.declare_parameter('lidar_topic', '/swan/lidar_detections')
        self.declare_parameter('out_topic', '/swan/cam_detections')
        self.declare_parameter('yolo_timeout', 1.5)      # 이 시간 지난 YOLO 결과는 폐기 [s]
        g = lambda n: self.get_parameter(n).value
        self.timeout = g('yolo_timeout')

        self.classes = []          # 최근 YOLO 가 본 클래스 이름들
        self.stamp = None
        self.create_subscription(String, g('yolo_topic'), self.on_yolo, 10)
        self.create_subscription(DetectionArray, g('lidar_topic'), self.on_lidar, 10)
        self.pub = self.create_publisher(DetectionArray, g('out_topic'), 10)
        self.get_logger().info(
            'YOLO 어댑터: dayoung /yolo/detected_objects → /swan/cam_detections '
            '(LiDAR 최근접 각도 부여, 사람 우선)')

    def on_yolo(self, m: String):
        # "Detected: person, car" → ['person','car']
        txt = m.data.split(':', 1)[-1] if ':' in m.data else m.data
        self.classes = [x.strip().lower() for x in txt.split(',') if x.strip()]
        self.stamp = self.get_clock().now()

    def _fresh(self):
        if self.stamp is None:
            return False
        return (self.get_clock().now() - self.stamp).nanoseconds / 1e9 < self.timeout

    def _priority(self, name):
        oname = NAME2OURS[name][1]
        return PRIORITY.index(oname) if oname in PRIORITY else 99

    def on_lidar(self, msg: DetectionArray):
        out = DetectionArray()
        out.header = msg.header
        out.img_w, out.img_h = msg.img_w, msg.img_h

        if self._fresh() and self.classes and msg.detections:
            # 관심 클래스만 + 안전 우선순위 정렬 (중복 제거)
            cls = sorted(set(c for c in self.classes if c in NAME2OURS), key=self._priority)
            # LiDAR 물체를 가까운 순으로 → 클래스와 1:1 배정
            obst = sorted(msg.detections, key=lambda d: d.distance_m)
            for i, c in enumerate(cls):
                if i >= len(obst):
                    break            # 물체보다 클래스가 많으면 나머지는 버림
                cid, cname = NAME2OURS[c]
                o = obst[i]
                d = Detection()
                d.class_id, d.class_name = cid, cname
                d.confidence = 0.5                       # dayoung conf 임계값(0.5)
                d.distance_m, d.angle_rad = o.distance_m, o.angle_rad   # LiDAR 각도 차용
                d.risk_level = o.risk_level
                out.detections.append(d)

        self.pub.publish(out)        # 없어도 빈 배열 발행(fusion 최신성 유지)


def main():
    rclpy.init()
    rclpy.spin(YoloAdapter())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
