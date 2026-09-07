#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class DrivingState(Enum):
    MANUAL = "MANUAL"
    STOPPED = "STOPPED"
    WAITING_FOR_NAV = "WAITING_FOR_NAV"
    AVOIDING = "AVOIDING"
    REPLANNING = "REPLANNING"
    BLOCKED = "BLOCKED"


class NavAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_avoidance_node")

        # --------------------------------------------------------------
        # 일반 주행 안전 거리
        # --------------------------------------------------------------


        # 장애물 접근 정지 및 자동 회피 시작
        self.stop_distance = 0.55

        # 상태가 거리 경계에서 반복 전환되는 현상 방지
        self.hysteresis = 0.12


        # STOPPED 진입 후 Nav2 회피 시작까지 대기
        self.avoidance_start_delay = 0.10

        # 빨간 신호등 정지 명령 1회 요청
        self.traffic_stop_requested = False

        # --------------------------------------------------------------
        # LiDAR 장애물 감지 설정
        # --------------------------------------------------------------

        # 휠체어 전방 통로 절반 폭
        self.corridor_half_width = 0.45

        # Supervisor가 사용할 최대 탐지 거리
        self.max_detection_distance = 2.50

        # 노이즈 제거를 위한 최소 군집 점 개수
        self.minimum_cluster_points = 3

        # 같은 장애물 군집으로 판단할 전후 거리 차이
        self.cluster_tolerance = 0.12

        # 장애물이 한두 프레임 사라져도 마지막 값을 잠시 유지한다.
        # 회전 중 장애물이 통로 밖으로 순간 이탈하며 CLEAR가 되는 문제를 방지한다.
        self.obstacle_hold_duration = 0.60



        # --------------------------------------------------------------
        # 회피 중 새 장애물 판단 설정
        # --------------------------------------------------------------

        # 회피 중 이 거리 이하의 새 장애물이 일정 시간 지속되면 재계획
        self.emergency_replan_distance = 0.28

        # 최초 장애물이 이 거리보다 멀어진 상태가 일정 시간 지속돼야
        # 새 장애물 감지를 활성화한다.
        self.replan_clear_distance = 0.55

        # 기존 장애물에서 벗어났다고 인정하기 위한 연속 시간
        self.replan_clear_duration = 0.70

        # 새 장애물이 실제로 지속된다고 인정하기 위한 연속 시간
        self.emergency_confirm_duration = 0.25

        # 목표 승인 직후 기존 장애물을 새 장애물로 보지 않도록 최소 대기
        self.replan_minimum_arm_delay = 0.80

        # 자동 재계획 최대 횟수
        self.maximum_replan_count = 3

        # --------------------------------------------------------------
        # Nav2 목표 설정
        # --------------------------------------------------------------

        # 현재 위치 기준 목표 거리 후보
        # 첫 후보 실패 시 다른 거리로 다시 시도한다.
        self.goal_distance_candidates = [
            2.8,
            3.2,
            2.4,
        ]

        # --------------------------------------------------------------
        # 입력 타임아웃
        # --------------------------------------------------------------

        self.user_cmd_timeout = 0.50
        self.nav_cmd_timeout = 0.50
        self.odom_timeout = 1.00
        self.scan_timeout = 2.00

        # 실제 속도 출력 주기: 20Hz
        self.output_period = 0.05

        # --------------------------------------------------------------
        # 현재 상태
        # --------------------------------------------------------------

        self.state = DrivingState.MANUAL
        self.c_avoidance_requested = False

        # 필터링 후 Supervisor가 사용하는 전방 거리
        self.front_distance = math.inf

        # 이번 scan에서 직접 측정된 원시 거리
        self.raw_front_distance = math.inf

        # 마지막으로 실제 장애물을 확인한 거리와 시각
        self.last_detected_distance = math.inf
        self.last_obstacle_seen_time = None

        # 최근 메시지
        self.latest_user_cmd = Twist()
        self.latest_nav_cmd = Twist()

        self.last_user_cmd_time = None
        self.last_nav_cmd_time = None
        self.last_scan_time = None
        self.last_odom_time = None

        # LiDAR timeout 상태 기록
        self.scan_timed_out = False

        # --------------------------------------------------------------
        # 현재 odom 위치
        # --------------------------------------------------------------

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # --------------------------------------------------------------
        # 회피 기준 정보
        # --------------------------------------------------------------

        # 최초 장애물 진입 당시 원래 진행 방향
        self.original_travel_yaw = 0.0

        # 현재 목표 계산의 시작 위치
        self.goal_start_x = 0.0
        self.goal_start_y = 0.0

        self.avoidance_pose_saved = False
        self.stopped_since = None

        # --------------------------------------------------------------
        # Nav2 목표 상태
        # --------------------------------------------------------------

        self.goal_candidate_index = 0
        self.goal_request_in_progress = False
        self.goal_handle = None
        self.goal_cancel_requested = False

        self.replan_requested = False
        self.replan_count = 0

        # 회피 중 기존 장애물과 새 장애물 구분
        self.emergency_replan_armed = False
        self.nav_goal_accepted_time = None
        self.clear_condition_since = None
        self.emergency_condition_since = None

        # 로그 출력 상태
        self.last_logged_state: Optional[DrivingState] = None
        self.last_logged_distance: Optional[float] = None

        # --------------------------------------------------------------
        # ROS 통신
        # --------------------------------------------------------------

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan_filtered",
            self.scan_callback,
            10,
        )

        self.situation_subscription = self.create_subscription(
            String,
            "/driving_situation",
            self.situation_callback,
            10,
        )

        self.user_cmd_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_user",
            self.user_cmd_callback,
            10,
        )

        self.nav_cmd_subscription = self.create_subscription(
            Twist,
            "/cmd_vel_nav",
            self.nav_cmd_callback,
            10,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        # 실제 휠체어 명령은 이 노드 하나만 발행한다.
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            "/model/wheelchair/cmd_vel",
            10,
        )

        self.avoidance_status_publisher = self.create_publisher(
            String,
            "/avoidance_status",
            10,
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        self.output_timer = self.create_timer(
            self.output_period,
            self.output_callback,
        )

        self.supervisor_timer = self.create_timer(
            0.10,
            self.supervisor_callback,
        )

        self.monitor_timer = self.create_timer(
            0.20,
            self.monitor_callback,
        )

        self.get_logger().info(
            "Driving supervisor started: "
            "stable LiDAR detection + manual safety + Nav2 avoidance"
        )

    # ==============================================================
    # 입력 콜백
    # ==============================================================

    def situation_callback(self, msg: String) -> None:
        situation = msg.data.strip().upper()

        # --------------------------------------------------------------
        # 빨간 신호등
        # --------------------------------------------------------------

        if situation == "RED":
            # 일반 수동 주행 중일 때만 정지 명령을 1회 요청한다.
            if self.state == DrivingState.MANUAL:
                self.traffic_stop_requested = True

                self.get_logger().info(
                    "Traffic light RED - one-time stop"
                )

            return

        # 초록불은 제어에 개입하지 않는다.
        if situation == "GREEN":
            return

        # --------------------------------------------------------------
        # 기존 장애물 C 처리
        # --------------------------------------------------------------

        if situation != "C":
            return

        if self.state in (
            DrivingState.STOPPED,
            DrivingState.WAITING_FOR_NAV,
            DrivingState.AVOIDING,
            DrivingState.REPLANNING,
            DrivingState.BLOCKED,
        ):
            return

        self.get_logger().info(
            "C situation received - starting avoidance"
        )

        self.c_avoidance_requested = True
        self.enter_stopped_state()
    
    def user_cmd_callback(self, msg: Twist) -> None:
        self.latest_user_cmd = self.copy_twist(msg)
        self.last_user_cmd_time = self.get_clock().now()

    def nav_cmd_callback(self, msg: Twist) -> None:
        self.latest_nav_cmd = self.copy_twist(msg)
        self.last_nav_cmd_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation

        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )

        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )

        self.current_yaw = math.atan2(
            sin_yaw,
            cos_yaw,
        )

        self.last_odom_time = self.get_clock().now()
        self.odom_received = True

    def scan_callback(self, msg: LaserScan) -> None:
        now = self.get_clock().now()

        self.last_scan_time = now

        if self.scan_timed_out:
            self.scan_timed_out = False
            self.get_logger().info(
                "LiDAR data recovered"
            )

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


    # ==============================================================
    # LiDAR 처리
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

            if x <= 0.0:
                continue

            if x > self.max_detection_distance:
                continue

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
                return cluster[len(cluster) // 2]

        return None

    def get_stabilized_front_distance(self) -> float:
        """
        장애물이 한두 scan 동안 사라져도 마지막 거리를 잠시 유지한다.

        단, 실제 측정이 다시 들어오면 즉시 새로운 값을 사용한다.
        """
        if math.isfinite(self.raw_front_distance):
            return self.raw_front_distance

        if self.last_obstacle_seen_time is None:
            return math.inf

        obstacle_age = self.get_age_seconds(
            self.last_obstacle_seen_time
        )

        if obstacle_age <= self.obstacle_hold_duration:
            return self.last_detected_distance

        return math.inf

    # ==============================================================
    # 일반 주행 상태
    # ==============================================================



    def enter_stopped_state(self) -> None:
        if self.state != DrivingState.STOPPED:
            self.stopped_since = self.get_clock().now()
            self.avoidance_pose_saved = False

            self.goal_candidate_index = 0
            self.goal_request_in_progress = False
            self.goal_handle = None
            self.goal_cancel_requested = False

            self.replan_requested = False
            self.replan_count = 0

            self.emergency_replan_armed = False
            self.nav_goal_accepted_time = None
            self.clear_condition_since = None
            self.emergency_condition_since = None

        self.change_state(DrivingState.STOPPED)

    # ==============================================================
    # Supervisor
    # ==============================================================

    def supervisor_callback(self) -> None:
        if self.state == DrivingState.STOPPED:
            self.handle_stopped_state()

        elif self.state == DrivingState.AVOIDING:
            self.handle_avoiding_state()

        elif self.state == DrivingState.REPLANNING:
            self.handle_replanning_state()

    def handle_stopped_state(self) -> None:
        if self.stopped_since is None:
            self.stopped_since = self.get_clock().now()
            return

        if not self.c_avoidance_requested:
            return

        if not self.is_odom_fresh():
            self.get_logger().warn(
                "Cannot start avoidance: odometry is unavailable",
                throttle_duration_sec=2.0,
            )
            return

        if (
            self.get_age_seconds(self.stopped_since)
            < self.avoidance_start_delay
        ):
            return

        if not self.avoidance_pose_saved:
            self.save_initial_avoidance_pose()

        if self.goal_request_in_progress:
            return

        if self.goal_handle is not None:
            return

        if not self.navigate_client.server_is_ready():
            self.get_logger().warn(
                "Waiting for /navigate_to_pose action server...",
                throttle_duration_sec=2.0,
            )
            return

        self.send_current_candidate_goal()

    def save_initial_avoidance_pose(self) -> None:
        self.goal_start_x = self.current_x
        self.goal_start_y = self.current_y
        self.original_travel_yaw = self.current_yaw

        self.avoidance_pose_saved = True

        self.get_logger().info(
            "Avoidance start pose saved: "
            f"x={self.goal_start_x:.2f}, "
            f"y={self.goal_start_y:.2f}, "
            f"yaw={self.original_travel_yaw:.2f}"
        )

    # ==============================================================
    # Nav2 목표 전송
    # ==============================================================

    def send_current_candidate_goal(self) -> None:
        if (
            self.goal_candidate_index
            >= len(self.goal_distance_candidates)
        ):
            self.enter_blocked_state(
                "No feasible avoidance goal remains"
            )
            return

        goal_distance = self.goal_distance_candidates[
            self.goal_candidate_index
        ]

        goal_x = (
            self.goal_start_x
            + goal_distance
            * math.cos(self.original_travel_yaw)
        )

        goal_y = (
            self.goal_start_y
            + goal_distance
            * math.sin(self.original_travel_yaw)
        )

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = "odom"
        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.position.z = 0.0

        half_yaw = self.original_travel_yaw / 2.0

        goal_msg.pose.pose.orientation.z = math.sin(half_yaw)
        goal_msg.pose.pose.orientation.w = math.cos(half_yaw)

        self.goal_request_in_progress = True
        self.change_state(DrivingState.WAITING_FOR_NAV)

        self.get_logger().info(
            "Sending avoidance goal "
            f"{self.goal_candidate_index + 1}/"
            f"{len(self.goal_distance_candidates)}: "
            f"distance={goal_distance:.2f}m, "
            f"goal=({goal_x:.2f}, {goal_y:.2f})"
        )

        send_future = self.navigate_client.send_goal_async(
            goal_msg
        )

        send_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future) -> None:
        self.goal_request_in_progress = False

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(
                f"Failed to send Nav2 goal: {error}"
            )
            self.try_next_goal_candidate()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                "Nav2 rejected avoidance goal"
            )
            self.try_next_goal_candidate()
            return

        self.goal_handle = goal_handle
        self.goal_cancel_requested = False

        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None

        self.emergency_replan_armed = False
        self.nav_goal_accepted_time = self.get_clock().now()
        self.clear_condition_since = None
        self.emergency_condition_since = None

        self.change_state(DrivingState.AVOIDING)

        self.get_logger().info(
            "Avoidance goal accepted by Nav2"
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    # ==============================================================
    # 회피 중 새 장애물 판단
    # ==============================================================

    def handle_avoiding_state(self) -> None:
        now = self.get_clock().now()

        if self.nav_goal_accepted_time is None:
            return

        goal_age = self.get_age_seconds(
            self.nav_goal_accepted_time
        )

        # 목표 승인 직후에는 기존 장애물 때문에 재계획하지 않는다.
        if goal_age < self.replan_minimum_arm_delay:
            self.clear_condition_since = None
            self.emergency_condition_since = None
            return

        if not self.emergency_replan_armed:
            clear_now = (
                not math.isfinite(self.raw_front_distance)
                or self.raw_front_distance
                >= self.replan_clear_distance
            )

            if clear_now:
                if self.clear_condition_since is None:
                    self.clear_condition_since = now

                elif (
                    self.get_age_seconds(
                        self.clear_condition_since
                    )
                    >= self.replan_clear_duration
                ):
                    self.emergency_replan_armed = True
                    self.clear_condition_since = None
                    self.emergency_condition_since = None

                    self.get_logger().info(
                        "Emergency replanning armed: "
                        "previous obstacle continuously cleared"
                    )

            else:
                self.clear_condition_since = None

            return

        emergency_now = (
            math.isfinite(self.raw_front_distance)
            and self.raw_front_distance
            <= self.emergency_replan_distance
        )

        if not emergency_now:
            self.emergency_condition_since = None
            return

        if self.emergency_condition_since is None:
            self.emergency_condition_since = now
            return

        if (
            self.get_age_seconds(
                self.emergency_condition_since
            )
            < self.emergency_confirm_duration
        ):
            return

        if self.goal_cancel_requested:
            return

        if self.replan_requested:
            return

        if self.replan_count >= self.maximum_replan_count:
            self.cancel_for_blocked_state(
                "Maximum automatic replan count reached"
            )
            return

        self.get_logger().warn(
            "Confirmed new obstacle during avoidance. "
            "Canceling the current goal and replanning."
        )

        self.publish_stop()

        self.replan_requested = True
        self.replan_count += 1

        self.emergency_replan_armed = False
        self.clear_condition_since = None
        self.emergency_condition_since = None

        self.cancel_current_goal()

    # ==============================================================
    # 목표 취소 및 재계획
    # ==============================================================

    def cancel_current_goal(self) -> None:
        if self.goal_handle is None:
            self.enter_blocked_state(
                "Cannot cancel Nav2 goal: goal handle is unavailable"
            )
            return

        if self.goal_cancel_requested:
            return

        self.goal_cancel_requested = True

        cancel_future = self.goal_handle.cancel_goal_async()

        cancel_future.add_done_callback(
            self.cancel_response_callback
        )

    def cancel_for_blocked_state(self, reason: str) -> None:
        self.replan_requested = False

        if self.goal_handle is None:
            self.enter_blocked_state(reason)
            return

        self.get_logger().error(reason)
        self.cancel_current_goal()

    def cancel_response_callback(self, future) -> None:
        try:
            future.result()

            self.get_logger().info(
                "Nav2 goal cancel request accepted"
            )

        except Exception as error:
            self.get_logger().error(
                f"Failed to cancel Nav2 goal: {error}"
            )

            self.goal_cancel_requested = False
            self.replan_requested = False
            self.goal_handle = None

            self.enter_blocked_state(
                "Nav2 goal cancellation failed"
            )

    def navigation_result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
            status = wrapped_result.status

        except Exception as error:
            self.get_logger().error(
                f"Failed to receive Nav2 result: {error}"
            )

            self.goal_handle = None
            self.goal_request_in_progress = False

            if self.replan_requested:
                self.change_state(DrivingState.REPLANNING)
            else:
                self.try_next_goal_candidate()

            return

        self.goal_handle = None
        self.goal_request_in_progress = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.handle_navigation_success()
            return

        if self.goal_cancel_requested:
            self.goal_cancel_requested = False

            self.latest_nav_cmd = Twist()
            self.last_nav_cmd_time = None

            if self.replan_requested:
                self.change_state(DrivingState.REPLANNING)

                self.get_logger().info(
                    "Previous Nav2 goal canceled. "
                    f"Replan count={self.replan_count}/"
                    f"{self.maximum_replan_count}"
                )

            else:
                self.enter_blocked_state(
                    "Avoidance canceled without a replan request"
                )

            return

        self.get_logger().warn(
            f"Nav2 avoidance failed: status={status}"
        )

        self.try_next_goal_candidate()

    def handle_replanning_state(self) -> None:
        if not self.replan_requested:
            self.enter_blocked_state(
                "REPLANNING entered without a request"
            )
            return

        if self.goal_request_in_progress:
            return

        if self.goal_handle is not None:
            return

        if not self.is_odom_fresh():
            self.get_logger().warn(
                "Cannot replan: odometry is unavailable",
                throttle_duration_sec=2.0,
            )
            return

        if not self.navigate_client.server_is_ready():
            self.get_logger().warn(
                "Waiting for /navigate_to_pose during replanning...",
                throttle_duration_sec=2.0,
            )
            return

        # 새 장애물이므로 후보를 처음부터 다시 시작한다.
        self.goal_candidate_index = 0

        # 현재 위치 기준으로 2m 안쪽 목표를 다시 생성한다.
        # 방향은 최초 수동 주행 방향을 계속 유지한다.
        self.goal_start_x = self.current_x
        self.goal_start_y = self.current_y

        self.replan_requested = False

        self.get_logger().info(
            "Replanning from current position: "
            f"x={self.goal_start_x:.2f}, "
            f"y={self.goal_start_y:.2f}, "
            f"original_yaw={self.original_travel_yaw:.2f}"
        )

        self.send_current_candidate_goal()

    def try_next_goal_candidate(self) -> None:
        self.goal_handle = None
        self.goal_request_in_progress = False

        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None

        self.goal_candidate_index += 1

        if (
            self.goal_candidate_index
            >= len(self.goal_distance_candidates)
        ):
            self.enter_blocked_state(
                "No feasible avoidance goal remains"
            )
            return

        self.send_current_candidate_goal()

    # ==============================================================
    # 성공 및 실패 처리
    # ==============================================================

    def handle_navigation_success(self) -> None:
        self.get_logger().info(
            "Avoidance completed successfully"
        )

        self.publish_avoidance_status("COMPLETED")

        # 회피 전 teleop 명령이 다시 적용되지 않도록 초기화한다.
        self.latest_user_cmd = Twist()
        self.last_user_cmd_time = None

        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None

        self.c_avoidance_requested = False
        self.reset_avoidance_session()
        self.change_state(DrivingState.MANUAL)

    def enter_blocked_state(self, reason: str) -> None:
        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None

        self.goal_handle = None
        self.goal_request_in_progress = False
        self.goal_cancel_requested = False
        self.replan_requested = False

        self.change_state(DrivingState.BLOCKED)

        self.get_logger().error(
            f"{reason}. Manual reverse and rotation are available."
        )

    def reset_avoidance_session(self) -> None:
        self.stopped_since = None
        self.avoidance_pose_saved = False

        self.goal_candidate_index = 0
        self.goal_request_in_progress = False
        self.goal_handle = None
        self.goal_cancel_requested = False

        self.replan_requested = False
        self.replan_count = 0

        self.emergency_replan_armed = False
        self.nav_goal_accepted_time = None
        self.clear_condition_since = None
        self.emergency_condition_since = None

    # ==============================================================
    # 최종 속도 출력
    # ==============================================================

    def output_callback(self) -> None:
        if not self.is_scan_fresh():
            self.publish_stop()
            return

            # 빨간 신호등 정지 요청은 한 번만 실행한다.
        if self.traffic_stop_requested:
            self.publish_stop()
            self.traffic_stop_requested = False
            return

        if self.state == DrivingState.MANUAL:
            if not self.is_user_cmd_fresh():
                self.publish_stop()
                return

            self.cmd_vel_publisher.publish(
                self.copy_twist(self.latest_user_cmd)
            )
            return

        if self.state == DrivingState.STOPPED:
            if not self.is_user_cmd_fresh():
                self.publish_stop()
                return

            self.cmd_vel_publisher.publish(
                self.make_recovery_cmd(
                    self.latest_user_cmd
                )
            )
            return

        if self.state == DrivingState.AVOIDING:
            if not self.is_nav_cmd_fresh():
                self.publish_stop()
                return

            self.cmd_vel_publisher.publish(
                self.copy_twist(self.latest_nav_cmd)
            )
            return

        if self.state == DrivingState.BLOCKED:
            if not self.is_user_cmd_fresh():
                self.publish_stop()
                return

            # 자동 회피가 실패해도 전진만 막고
            # 후진과 제자리 회전은 허용한다.
            self.cmd_vel_publisher.publish(
                self.make_recovery_cmd(
                    self.latest_user_cmd
                )
            )
            return

        # WAITING_FOR_NAV와 REPLANNING에서는 완전 정지
        self.publish_stop()



    def make_recovery_cmd(
        self,
        user_cmd: Twist,
    ) -> Twist:
        """
        전진만 차단하고 후진과 회전은 허용한다.
        """
        output_cmd = self.copy_twist(user_cmd)

        if output_cmd.linear.x > 0.0:
            output_cmd.linear.x = 0.0

        return output_cmd

    # ==============================================================
    # 상태 확인
    # ==============================================================

    def is_scan_fresh(self) -> bool:
        if self.last_scan_time is None:
            return False

        fresh = (
            self.get_age_seconds(self.last_scan_time)
            <= self.scan_timeout
        )

        if not fresh and not self.scan_timed_out:
            self.scan_timed_out = True

            self.get_logger().error(
                "LiDAR timeout: output is stopped"
            )

        return fresh

    def is_user_cmd_fresh(self) -> bool:
        if self.last_user_cmd_time is None:
            return False

        return (
            self.get_age_seconds(self.last_user_cmd_time)
            <= self.user_cmd_timeout
        )

    def is_nav_cmd_fresh(self) -> bool:
        if self.last_nav_cmd_time is None:
            return False

        return (
            self.get_age_seconds(self.last_nav_cmd_time)
            <= self.nav_cmd_timeout
        )

    def is_odom_fresh(self) -> bool:
        if self.last_odom_time is None:
            return False

        return (
            self.get_age_seconds(self.last_odom_time)
            <= self.odom_timeout
        )

    def get_age_seconds(self, recorded_time) -> float:
        return (
            self.get_clock().now()
            - recorded_time
        ).nanoseconds / 1_000_000_000.0

    # ==============================================================
    # 로그
    # ==============================================================

    def monitor_callback(self) -> None:
        if self.last_scan_time is None:
            self.get_logger().warn(
                "Waiting for /scan_filtered...",
                throttle_duration_sec=2.0,
            )
            return

        if not self.is_scan_fresh():
            return

        if not self.odom_received:
            self.get_logger().warn(
                "Waiting for /odom...",
                throttle_duration_sec=2.0,
            )

        elif not self.is_odom_fresh():
            self.get_logger().warn(
                "Odometry timeout",
                throttle_duration_sec=2.0,
            )

        if not self.should_log_status():
            return

        if math.isinf(self.front_distance):
            distance_text = "CLEAR"
        else:
            distance_text = f"{self.front_distance:.2f}m"

        if math.isinf(self.raw_front_distance):
            raw_text = "CLEAR"
        else:
            raw_text = f"{self.raw_front_distance:.2f}m"

        self.get_logger().info(
            f"state={self.state.value}, "
            f"front_distance={distance_text}, "
            f"raw={raw_text}, "
            f"pose=({self.current_x:.2f}, "
            f"{self.current_y:.2f}, "
            f"yaw={self.current_yaw:.2f})"
        )

        self.last_logged_state = self.state
        self.last_logged_distance = self.front_distance

    def should_log_status(self) -> bool:
        if self.last_logged_state != self.state:
            return True

        if self.last_logged_distance is None:
            return True

        if math.isinf(self.front_distance):
            return not math.isinf(
                self.last_logged_distance
            )

        if math.isinf(self.last_logged_distance):
            return True

        return abs(
            self.front_distance
            - self.last_logged_distance
        ) >= 0.10

    def publish_avoidance_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.avoidance_status_publisher.publish(msg)

        self.get_logger().info(
            f"AVOIDANCE STATUS: {status}"
        )

    def change_state(
        self,
        new_state: DrivingState,
    ) -> None:
        if self.state == new_state:
            return

        previous_state = self.state
        self.state = new_state

        self.log_state_change(
            previous_state,
            new_state,
        )

        if new_state in (
            DrivingState.STOPPED,
            DrivingState.WAITING_FOR_NAV,
            DrivingState.AVOIDING,
            DrivingState.REPLANNING,
            DrivingState.BLOCKED,
        ):
            self.publish_avoidance_status(
                new_state.value
            )

    def log_state_change(
        self,
        previous_state: DrivingState,
        new_state: DrivingState,
    ) -> None:
        self.get_logger().info(
            "STATE CHANGED: "
            f"{previous_state.value} -> {new_state.value}"
        )

    # ==============================================================
    # 공통 함수
    # ==============================================================

    def publish_stop(self) -> None:
        if not rclpy.ok():
            return

        self.cmd_vel_publisher.publish(Twist())

    @staticmethod
    def copy_twist(source: Twist) -> Twist:
        copied = Twist()

        copied.linear.x = source.linear.x
        copied.linear.y = source.linear.y
        copied.linear.z = source.linear.z

        copied.angular.x = source.angular.x
        copied.angular.y = source.angular.y
        copied.angular.z = source.angular.z

        return copied


def main(args=None) -> None:
    rclpy.init(args=args)

    node = NavAvoidanceNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.publish_stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()