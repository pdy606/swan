#!/usr/bin/env python3
"""
[인지 노드 - 스텁]  ★ 비전팀이 채울 부분 ★
지금은 YOLO 대신 '가짜 시나리오'를 발행: 처음엔 장애물 없음 →
잠시 후 킥보드가 오른쪽 앞에서 점점 가까워지는 상황을 흉내낸다.
나중에: 이미지 구독 → YOLO 추론 → 거리/각도 추정 → 같은 토픽으로 발행.
"""
import math
import rclpy
from rclpy.node import Node
from swan_interfaces.msg import Detection, DetectionArray


class PerceptionStub(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.pub = self.create_publisher(DetectionArray, '/swan/detections', 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)   # 10 Hz
        self.get_logger().info('perception(stub) 시작: 가짜 장애물 시나리오 발행')

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        msg = DetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.img_w, msg.img_h = 640, 480

        # t<4s: 장애물 없음 → 빈 배열 (정상주행)
        # t>=4s: 킥보드가 4m에서 접근(0.3m/s), 오른쪽(각도 -0.2rad)
        if t >= 4.0:
            dist = max(0.4, 4.0 - 0.3 * (t - 4.0))
            d = Detection()
            d.class_id, d.class_name = 0, 'kickboard'
            d.confidence = 0.9
            d.bbox_x, d.bbox_y, d.bbox_w, d.bbox_h = 0.6, 0.55, 0.15, 0.25
            d.distance_m = dist
            d.angle_rad = -0.2                       # 오른쪽
            d.risk_level = 2 if dist < 2.0 else 1
            msg.detections.append(d)

        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(PerceptionStub())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
