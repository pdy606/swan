#!/usr/bin/env python3
"""
[신호 행동 노드 — B 상황]  ★ 의미 기반 행동 전담
'거리로 피하는' C 와 달리, **의미로 판단**해야 하는 상황(B)을 처리한다.
대표 예: 신호등 — 빨강=정지, 초록=진행.

입력:
  /swan/detections   (fusion, DetectionArray)  — 신호등이 LiDAR 거리와 결합돼 옴
                                                 (다이어그램 ⑤: "정면 6m 빨간불")
출력:
  /driving_situation (String)  — "B"(빨강=정지)   ※ A(평상) 없음, 초록은 해제
  /signal_action     (String)  — 사람이 읽는 행동 ("STOP@red_light 6.0m" 등)

규칙:
  fusion 이 준 traffic_light_red → "B" (거리 포함 → 정지선 기반 감속/정지 가능)
  traffic_light_green            → B 해제(진행), /driving_situation 발행 안 함
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from swan_interfaces.msg import DetectionArray

TRAFFIC_LIGHT_ID = 12


class SignalNode(Node):
    def __init__(self):
        super().__init__('signal_node')
        g = lambda n: self.get_parameter(n).value
        self.declare_parameter('det_topic', '/swan/detections')
        self.state = None      # 'B'(정지) / 'go' / None
        self.create_subscription(DetectionArray, g('det_topic'), self.on_det, 10)
        self.sit_pub = self.create_publisher(String, '/driving_situation', 10)
        self.act_pub = self.create_publisher(String, '/signal_action', 10)
        self.get_logger().info('signal_node(B) 시작: fusion 의 신호등(거리 포함) → 빨강=정지/초록=진행')

    def on_det(self, msg: DetectionArray):
        # fusion 이 결합해 준 신호등 검출 찾기 (거리 포함)
        sig = next((d for d in msg.detections if d.class_id == TRAFFIC_LIGHT_ID), None)
        if sig is None:
            return                       # 신호 없음 → A/C(다른 노드)에 맡김

        red = 'red' in sig.class_name
        if red:
            act = f'STOP@red_light {sig.distance_m:.1f}m'
            if self.state != 'B':
                self.get_logger().info(f'빨강 신호 (정면 {sig.distance_m:.1f}m) → 정지 (B)')
                self.state = 'B'
            self.sit_pub.publish(String(data='B'))       # → 규원/제어: 감속·정지
            self.act_pub.publish(String(data=act))
        else:                            # 초록 → 진행(B 해제). A 는 발행 안 함
            if self.state != 'go':
                self.get_logger().info('초록 신호 → 진행 (B 해제)')
                self.state = 'go'
            self.act_pub.publish(String(data=f'GO@green_light {sig.distance_m:.1f}m'))


def main():
    rclpy.init()
    rclpy.spin(SignalNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
