#!/usr/bin/env python3
"""
[센서 퓨전 노드]  LiDAR(정확한 거리) + 카메라(클래스) → 통합 /swan/detections
- 입력: /swan/lidar_detections (거리·각도, class=unknown)
        /swan/cam_detections   (클래스·bbox·대략거리·각도)
- 매칭: 각도가 가까운 것끼리 = 같은 물체
        → 거리는 LiDAR(정확), 클래스는 카메라 로 결합
- 출력: /swan/detections (판단 노드가 쓰는 최종 계약)

규칙:
  LiDAR+카메라 매칭 → 거리=LiDAR, 클래스=카메라 (최선)
  LiDAR만          → 거리=LiDAR, 클래스=unknown (안전하게 회피)
  카메라만(신호등/미매칭 클래스) → 그대로 전달 (거리 부정확해도 클래스 중요)

※ 매칭 튜닝 노트:
  - 카메라 각도는 bbox 중심, LiDAR 각도는 클러스터 중심 → 물체가 크면 몇 도 어긋남.
    그래서 angle_match 를 넉넉히(0.30rad≈17°) 준다.
  - 카메라 인지 처리가 무거워 발행이 느릴 수 있음 → cam_timeout 넉넉히(2s).
  - 신호등(traffic_light_*)은 물리 장애물이 아니라 '신호'라 물체 매칭에서 제외.
"""
import rclpy
from rclpy.node import Node
from swan_interfaces.msg import Detection, DetectionArray

TRAFFIC_LIGHT_ID = 12   # camera_perception 이 신호등에 쓰는 class_id


def is_signal(det) -> bool:
    """신호등처럼 물리 장애물이 아닌 '신호' 검출인가 (물체 매칭 제외 대상)."""
    return det.class_id == TRAFFIC_LIGHT_ID or det.class_name.startswith('traffic_light')


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('angle_match', 0.30)   # 매칭 허용 각도차 [rad] ≈17°
        self.declare_parameter('cam_timeout', 2.0)    # 카메라 최신성 [s]
        self.declare_parameter('log_period', 2.0)     # 상태 로그 주기 [s] (0=끔)
        self.match = self.get_parameter('angle_match').value
        self.cam_to = self.get_parameter('cam_timeout').value
        self.log_period = self.get_parameter('log_period').value

        self.cam = []            # 최근 카메라 검출
        self.cam_stamp = None
        self.last_log = None
        self.create_subscription(DetectionArray, '/swan/lidar_detections', self.on_lidar, 10)
        self.create_subscription(DetectionArray, '/swan/cam_detections', self.on_cam, 10)
        self.pub = self.create_publisher(DetectionArray, '/swan/detections', 10)
        self.get_logger().info(
            f'fusion 시작: LiDAR거리+카메라클래스 결합 (angle_match={self.match}rad, '
            f'cam_timeout={self.cam_to}s)')

    def on_cam(self, msg: DetectionArray):
        self.cam = list(msg.detections)
        self.cam_stamp = self.get_clock().now()

    def cam_fresh(self):
        if self.cam_stamp is None:
            return False
        return (self.get_clock().now() - self.cam_stamp).nanoseconds / 1e9 < self.cam_to

    def on_lidar(self, msg: DetectionArray):
        out = DetectionArray()
        out.header = msg.header
        out.img_w, out.img_h = msg.img_w, msg.img_h
        fresh = self.cam_fresh()
        cam = self.cam if fresh else []
        # 물체 매칭 후보 = 신호가 아닌 카메라 검출
        obj_cam = [(i, c) for i, c in enumerate(cam) if not is_signal(c)]
        used_cam = set()
        matched = 0

        # 1) LiDAR 각 물체에 카메라 클래스 매칭 (각도 최근접)
        for ld in msg.detections:
            best, best_da = None, self.match
            for i, cd in obj_cam:
                if i in used_cam:
                    continue
                da = abs(cd.angle_rad - ld.angle_rad)
                if da < best_da:
                    best, best_da = i, da
            d = Detection()
            d.distance_m, d.angle_rad = ld.distance_m, ld.angle_rad   # 거리=LiDAR(정확)
            d.risk_level = ld.risk_level
            if best is not None:                                       # 클래스=카메라
                used_cam.add(best)
                matched += 1
                d.class_id, d.class_name = cam[best].class_id, cam[best].class_name
                d.confidence = cam[best].confidence
                d.bbox_x, d.bbox_y = cam[best].bbox_x, cam[best].bbox_y
                d.bbox_w, d.bbox_h = cam[best].bbox_w, cam[best].bbox_h
            else:
                d.class_id, d.class_name, d.confidence = 99, 'unknown', 1.0
            out.detections.append(d)

        # 2) 카메라만 본 것 (신호등 + 미매칭 클래스) 그대로 추가 → 클래스 정보 보존
        for i, cd in enumerate(cam):
            if i not in used_cam:
                out.detections.append(cd)

        self.pub.publish(out)
        self._maybe_log(len(msg.detections), len(cam), fresh, matched)

    def _maybe_log(self, n_lidar, n_cam, fresh, matched):
        if self.log_period <= 0:
            return
        now = self.get_clock().now()
        if self.last_log and (now - self.last_log).nanoseconds / 1e9 < self.log_period:
            return
        self.last_log = now
        state = 'fresh' if fresh else ('STALE' if self.cam_stamp else '없음')
        self.get_logger().info(
            f'lidar={n_lidar} cam={n_cam}({state}) → 매칭={matched}, unknown={n_lidar - matched}')


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
