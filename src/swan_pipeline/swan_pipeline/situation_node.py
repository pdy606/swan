#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SituationNode(Node):

    def __init__(self) -> None:
        super().__init__("situation_node")

        # --------------------------------------------------------------
        # 장애물 접근 / 회피 시작 설정
        # --------------------------------------------------------------

        # 장애물 접근 시 C 상황 발생 거리
        self.stop_distance = 0.55

        # 거리 경계에서 C 상태가 반복되는 현상 방지
        self.hysteresis = 0.12

        # --------------------------------------------------------------
        # LiDAR 장애물 감지 설정
        # nav_avoidance_node의 기존 판단 방식과 동일
        # --------------------------------------------------------------

        # 휠체어 전방 통로 절반 폭
        self.corridor_half_width = 0.45

        # 판단에 사용할 최대 탐지 거리
        self.max_detection_distance = 2.50

        # 노이즈 제거를 위한 최소 군집 점 개수
        self.minimum_cluster_points = 3

        # 같은 장애물 군집으로 판단할 전후 거리 차이
        self.cluster_tolerance = 0.12

        # 장애물이 한두 프레임 사라져도 마지막 값을 잠시 유지
        self.obstacle_hold_duration = 0.60

        # --------------------------------------------------------------
        # 현재 장애물 상태
        # --------------------------------------------------------------

        # 필터링 후 판단에 사용하는 전방 거리
        self.front_distance = math.inf

        # 현재 scan에서 직접 측정된 거리
        self.raw_front_distance = math.inf

        # 마지막으로 확인한 장애물 정보
        self.last_detected_distance = math.inf
        self.last_obstacle_seen_time = None

        # 같은 장애물에 C를 반복 발행하지 않도록 사용
        self.c_active = False

        # --------------------------------------------------------------
        # ROS 통신
        # --------------------------------------------------------------

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan_filtered",
            self.scan_callback,
            10,
        )

        self.situation_publisher = self.create_publisher(
            String,
            "/driving_situation",
            10,
        )

        self.get_logger().info(
            "Situation node started: "
            "stable LiDAR detection -> driving situation"
        )

    # ==============================================================
    # LiDAR 입력
    # ==============================================================

    def scan_callback(self, msg: LaserScan) -> None:
        now = self.get_clock().now()

        corridor_distances = self.extract_corridor_distances(msg)

        detected_distance = self.find_nearest_cluster(
            corridor_distances
        )

        if detected_distance is None:
            self.raw_front_distance = math.inf

        else:
            self.raw_front_distance = detected_distance
            self.last_detected_distance = detected_distance
            self.last_obstacle_seen_time = now

        self.front_distance = self.get_stabilized_front_distance()

        self.evaluate_driving_situation()

    # ==============================================================
    # LiDAR 처리
    # nav_avoidance_node와 동일
    # ==============================================================

    def extract_corridor_distances(
        self,
        msg: LaserScan,
    ) -> list[float]:

        distances: list[float] = []

        for index, distance in enumerate(msg.ranges):

            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            angle = msg.angle_min + (
                index * msg.angle_increment
            )

            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            # 휠체어 뒤쪽은 제외
            if x <= 0.0:
                continue

            # 최대 탐지 거리 밖은 제외
            if x > self.max_detection_distance:
                continue

            # 휠체어 진행 통로 밖은 제외
            if abs(y) > self.corridor_half_width:
                continue

            distances.append(x)

        distances.sort()

        return distances

    def find_nearest_cluster(
        self,
        distances: list[float],
    ) -> Optional[float]:

        if len(distances) < self.minimum_cluster_points:
            return None

        for start_index in range(len(distances)):

            cluster_start = distances[start_index]
            cluster: list[float] = []

            for distance in distances[start_index:]:

                if (
                    distance - cluster_start
                    <= self.cluster_tolerance
                ):
                    cluster.append(distance)

                else:
                    break

            if len(cluster) >= self.minimum_cluster_points:

                # 기존 코드와 동일하게 군집 중앙값 사용
                return cluster[len(cluster) // 2]

        return None

    def get_stabilized_front_distance(self) -> float:

        # 현재 scan에서 장애물이 확인됐으면 즉시 사용
        if math.isfinite(self.raw_front_distance):
            return self.raw_front_distance

        if self.last_obstacle_seen_time is None:
            return math.inf

        obstacle_age = self.get_age_seconds(
            self.last_obstacle_seen_time
        )

        # 한두 scan에서 장애물이 사라져도 기존 값 유지
        if obstacle_age <= self.obstacle_hold_duration:
            return self.last_detected_distance

        return math.inf

    # ==============================================================
    # 상황 판단
    # ==============================================================

    def evaluate_driving_situation(self) -> None:

        # 새로운 장애물이 회피 시작 거리 안으로 들어온 경우
        if not self.c_active:

            if (
                math.isfinite(self.front_distance)
                and self.front_distance <= self.stop_distance
            ):

                self.publish_situation("C")
                self.c_active = True

                self.get_logger().info(
                    "Obstacle detected: "
                    f"{self.front_distance:.2f}m -> C"
                )

            return

        # ----------------------------------------------------------
        # 기존 장애물이 충분히 멀어졌을 때 다음 C 허용
        # ----------------------------------------------------------

        clear_distance = (
            self.stop_distance
            + self.hysteresis
        )

        if (
            not math.isfinite(self.front_distance)
            or self.front_distance >= clear_distance
        ):

            self.c_active = False

            self.get_logger().info(
                "Obstacle cleared - "
                "ready for next C situation"
            )

    # ==============================================================
    # ROS 출력
    # ==============================================================

    def publish_situation(self, situation: str) -> None:

        msg = String()
        msg.data = situation

        self.situation_publisher.publish(msg)

    # ==============================================================
    # 공통
    # ==============================================================

    def get_age_seconds(self, recorded_time) -> float:

        return (
            self.get_clock().now()
            - recorded_time
        ).nanoseconds / 1_000_000_000.0


def main(args=None) -> None:

    rclpy.init(args=args)

    node = SituationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()