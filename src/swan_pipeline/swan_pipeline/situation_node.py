#!/usr/bin/env python3
"""
[상황 판단 노드 — C 상황 (회피)]
fusion 이 낸 /swan/detections 를 받아, 전방 장애물이 가까우면 회피(C)를 요청한다.
복잡한 상태머신 없이 C 트리거만 담당. (B=의미기반은 signal_node, 퍼셉션은 fusion)

입력:
  /swan/detections     (fusion, DetectionArray) — 거리·각도·클래스
  /avoidance_status    (규원, String)           — "COMPLETED"
출력:
  /driving_situation   (String)                 — "C"(회피) / "A"(정상)
                                                  → 규원 nav_avoidance_node 가 "C" 만 처리
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swan_interfaces.msg import DetectionArray


class SituationNode(Node):
    def __init__(self):
        super().__init__('situation_node')
        self.declare_parameter('trigger_dist', 1.5)     # 이 안에 장애물 → 회피(C)
        self.declare_parameter('front_angle', 0.5)      # 전방 판정 ±rad
        g = lambda n: self.get_parameter(n).value
        self.trigger = g('trigger_dist')
        self.front = g('front_angle')

        self.front_dist = 99.0
        self.avoiding = False
        self.create_subscription(DetectionArray, '/swan/detections', self.on_det, 10)
        self.create_subscription(String, '/avoidance_status', self.on_status, 10)
        self.pub = self.create_publisher(String, '/driving_situation', 10)
        self.create_timer(0.2, self.loop)               # 5 Hz
        self.get_logger().info(
            f'situation_node(C) 시작: 전방 {self.trigger}m 이내 장애물 → "C"')

    def on_det(self, msg: DetectionArray):
        best = 99.0
        for d in msg.detections:
            if d.class_id == 12:          # 신호등(의미기반)은 회피 대상 아님 → signal_node 담당
                continue
            if abs(d.angle_rad) <= self.front and d.distance_m < best:
                best = d.distance_m
        self.front_dist = best

    def on_status(self, m: String):
        if m.data.strip().upper() == 'COMPLETED' and self.avoiding:
            self.avoiding = False
            self.get_logger().info('avoidance COMPLETED → 정상 복귀')

    def loop(self):
        # A(위험없음=평상주행)는 발행하지 않음 — 장애물 있을 때 "C"만.
        if self.avoiding:
            return
        if self.front_dist < self.trigger:
            self.avoiding = True
            self.get_logger().info(f'C 발행 → Nav2 회피 (전방 {self.front_dist:.2f}m)')
            self.pub.publish(String(data='C'))


def main():
    rclpy.init()
    rclpy.spin(SituationNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
