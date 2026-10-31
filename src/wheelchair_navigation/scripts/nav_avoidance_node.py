#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool


class DrivingState(Enum):
    MANUAL = "MANUAL"
    STOPPED = "STOPPED"
    WAITING_FOR_NAV = "WAITING_FOR_NAV"
    AVOIDING = "AVOIDING"
    REPLANNING = "REPLANNING"
    BLOCKED = "BLOCKED"
    VERIFYING_CLEAR = "VERIFYING_CLEAR"


class NavAvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_avoidance_node")

        # 회피 시작 거리(현재 로컬에서 0.44m)는 situation_node 담당.
        # 이 노드는 C를 받은 뒤 회피 수행과 제어권 전환만 담당한다.
        self.avoidance_start_delay = 0.10

        # 자율주행 ON/OFF (기본 ON)
        # OFF에서는 Nav2 / 상황판단 / LiDAR 정지 조건을 모두 우회하고
        # 사용자의 /cmd_vel_user만 최종 모터 토픽으로 전달한다.
        self.autonomy_enabled = True
        # 이전 모드의 Nav2 비동기 응답이 현재 모드를 덮어쓰지 않도록 세대 구분
        self.control_epoch = 0
        # OFF 도중 전송되거나 취소 중인 예전 목표가 정리될 때까지 새 목표를 보류
        self.pending_nav_goal_accepts = 0
        self.detached_goal_keys = set()

        # OFF 중 무시한 C, 또는 OFF 직전 진행 중이던 회피를 기억한다.
        # situation_node는 C를 한 번만 발행할 수 있어 ON 복귀 시
        # 현재 LiDAR를 다시 평가해야 하는 경우에만 사용한다.
        self.recheck_obstacle_on_enable = False
        # situation_node의 현지 설정: 장애물 회피 시작 0.44m.
        # 일반 주행 중 C의 판단은 계속 situation_node가 담당한다.
        # 이 값은 OFF -> ON의 누락 C 복구에만 사용한다.
        self.enable_recheck_stop_distance = 0.44
        self.enable_recheck_release_distance = 0.56

        # --------------------------------------------------------------
        # 빨간 신호등 정지
        # --------------------------------------------------------------

        # RED를 받은 뒤 사용자가 전진 입력을 놓을 때까지 정지 유지
        self.traffic_stop_latched = False

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
        # LiDAR 기반의 짧은 측면 회피 목표
        # --------------------------------------------------------------

        # 현재 모델 model.sdf: lidar_link x=0.932m, y=0.
        # 센서 위치를 변경하면 이 값도 함께 수정할 것.
        self.lidar_offset_x = 0.932

        # 실제 footprint의 최대 측면 폭(half-width)은 0.39m.
        self.robot_half_width = 0.39
        # Nav2의 xy_goal_tolerance=0.40m를 고려해 목표 후보를
        # 장애물 가장자리에서 충분히 떨어뜨린다.
        # 0.20m의 측면 안전 여유 + 0.40m 목표 허용 오차 보정.
        # 임의로 차체 Footprint를 줄이지 않고 목표점만 조금 가깝게 조정.
        self.goal_clearance = 0.60
        self.goal_side_variants = (0.0, 0.12)
        self.goal_x_variants = (0.05, 0.20)
        self.max_goal_side_offset = 1.60  # 과도한 측면 우회를 제한
        self.max_goal_forward = 2.10      # 최초 회피 목표는 가까운 위치만
        self.max_goal_path_length = 3.20  # 너무 멀리 도는 경로 제외
        self.max_goal_path_side = 1.75    # 주행선으로부터의 최대 이탈 제한
        # 원래 진행방향에서 ±50도 이내의 유효 목표를 우선 선택한다.
        # 범위 안에 없으면 기존 최단 경로 순서를 그대로 사용한다.
        self.preferred_goal_angle_deg = 50.0
        # 선호 각도 내 유효 후보끼리 비교할 때만 측면 이탈에 작은 비용 부여.
        # 경로 길이(m) + 0.50 * 최대 측면 이탈(m). 장애물 안전 여유는 유지.
        self.lateral_deviation_weight = 0.50
        self.goal_plan_timeout = 15.0   # 경로 검증 지연 시 정지 유지

        # 목표 도착 후 전방에 새 위험이 없는지 확인하고 수동 복귀
        self.return_clear_distance = 0.65
        self.return_clear_duration = 0.50
        self.return_clear_since = None

        # BLOCKED 해제: 사용자의 실제 회피 조작과 공간 확보를 확인
        self.blocked_clear_distance = 0.65
        self.blocked_clear_duration = 0.60
        self.blocked_min_translation = 0.15
        self.blocked_min_rotation = 0.20  # rad
        self.blocked_pose = None
        self.blocked_clear_since = None

        # BLOCKED 복구 뒤에는 전진을 새로 입력해야 한다.
        # 정상 회피 성공 뒤에는 전진 잠금을 설정하지 않는다.
        self.forward_rearm_required = False
        self.forward_release_seen = False
        self.pending_block_reason = None

        # 이번 회피 시점의 LiDAR 스캔. 목표점 생성에만 사용한다.
        self.latest_scan = None

        # Nav2 검증을 통과한 (경로 길이, 목표 x, 목표 y, 최대 측면 이탈) 목표
        self.goal_candidates = []
        self.goal_plan_queue = []
        self.goal_plan_index = 0
        self.goal_plan_results = []
        self.goal_plan_in_progress = False
        self.goal_plan_started_at = None
        self.goal_plan_generation = 0

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

        # ros2 service call /set_autonomy std_srvs/srv/SetBool "{data: false}"
        self.autonomy_service = self.create_service(
            SetBool,
            "/set_autonomy",
            self.set_autonomy_callback,
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

        # 후보 목표의 경로 존재 여부와 길이를 Nav2에 맡긴다.
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
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
            "stable LiDAR detection + manual safety + Nav2 avoidance "
            "+ /set_autonomy (default ON)"
        )

    # ==============================================================
    # ON/OFF 서비스 및 기존 Nav2 목표 정리
    # ============================================================== 

    def set_autonomy_callback(self, request, response):
        """ON: 기존 회피 활성화 / OFF: 즉시 완전 수동 (전진 포함)."""
        enable = bool(request.data)
        if enable == self.autonomy_enabled:
            response.success = True
            response.message = "Already ON" if enable else "Already OFF"
            return response

        # 취소/결과가 느리게 도착하더라도 다른 모드 상태를 변경하지 못하게 한다.
        self.control_epoch += 1
        previous_goal = self.goal_handle
        if not enable:
            # OFF 직전 이미 진행 중이던 회피 / 실패 상태를 기억.
            # OFF 시 전부 초기화하지만, ON 시 동일 장애물이 아직 앞에
            # 있으면 situation_node의 C 재발행을 기다리지 않도록 한다.
            self.recheck_obstacle_on_enable = (
                self.recheck_obstacle_on_enable
                or self.c_avoidance_requested
                or self.state != DrivingState.MANUAL
            )
        self.autonomy_enabled = enable

        # 성공/실패/회피 대기/신호 정지/전진 잠금 등 이번 세션 상태 전부 리셋.
        # 최근 Teleop 입력은 그대로 보존해 OFF 직후 별도 키 없이 움직일 수 있다.
        self.reset_avoidance_session()
        self.c_avoidance_requested = False
        self.traffic_stop_latched = False
        self.forward_rearm_required = False
        self.forward_release_seen = False
        self.pending_block_reason = None
        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None
        self.change_state(DrivingState.MANUAL)

        if not enable:
            # 취소 완료 전에도 /cmd_vel_nav는 출력을 통과할 수 없다.
            # 기존 목표 취소는 비동기 실행; 늦게 승인되는 목표도 별도 취소함.
            if previous_goal is not None:
                self.cancel_detached_goal(previous_goal)
            self.publish_avoidance_status("AUTONOMY_OFF")
            self.get_logger().info(
                "Autonomy OFF: unrestricted manual teleop; "
                "previous avoidance state cleared"
            )
            self.publish_manual_or_stop()
            response.message = "Autonomy OFF: unrestricted manual control"
        else:
            self.publish_avoidance_status("AUTONOMY_ON")
            self.get_logger().info(
                "Autonomy ON: fresh MANUAL session; "
                + ("LiDAR recheck pending" if self.recheck_obstacle_on_enable
                   else "awaiting new C")
            )
            # 전방에 이미 장애물이 있는 경우 다음 scan_callback에서 재판단.
            response.message = "Autonomy ON: avoidance enabled"

        response.success = True
        return response

    def cancel_detached_goal(self, handle) -> None:
        """옛 Nav2 목표를 취소한다. 응답은 주행 상태를 변경하지 않는다."""
        if not handle.accepted:
            return
        key = bytes(handle.goal_id.uuid)
        if key in self.detached_goal_keys:
            return
        self.detached_goal_keys.add(key)

        try:
            done_future = handle.get_result_async()
            done_future.add_done_callback(
                lambda f, k=key: self.detached_goal_finished_callback(f, k)
            )
        except Exception as exc:
            self.get_logger().warn(f"Could not watch old Nav2 goal: {exc}")
            self.detached_goal_keys.discard(key)

        try:
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self.detached_cancel_response_callback(f)
            )
            self.get_logger().info("Cancellation requested for previous Nav2 goal")
        except Exception as exc:
            self.get_logger().error(f"Could not request old Nav2 goal cancel: {exc}")

    def detached_cancel_response_callback(self, future) -> None:
        # OFF 이후 늦게 들어온 cancel 응답은 절대 MANUAL/BLOCKED를 변경하지 않는다.
        try:
            answer = future.result()
            if not answer.goals_canceling:
                self.get_logger().warn(
                    "Old Nav2 goal cancel not confirmed; manual output remains isolated"
                )
        except Exception as exc:
            self.get_logger().warn(f"Old Nav2 cancel response error: {exc}")

    def detached_goal_finished_callback(self, future, key: bytes) -> None:
        self.detached_goal_keys.discard(key)
        try:
            status = future.result().status
            self.get_logger().info(f"Previous Nav2 goal finished (status={status})")
        except Exception as exc:
            self.get_logger().warn(f"Old Nav2 result unavailable: {exc}")

    def publish_manual_or_stop(self) -> None:
        if self.is_user_cmd_fresh():
            self.cmd_vel_publisher.publish(self.copy_twist(self.latest_user_cmd))
        else:
            self.publish_stop()

    # ==============================================================
    # 입력 콜백
    # ==============================================================

    def situation_callback(self, msg: String) -> None:
        situation = msg.data.strip().upper()
        # OFF면 RED / C / 기타 상황판단은 주행에 반영하지 않는다.
        # 단, C가 1회만 발행될 수 있으므로 ON 전환 후 재판단할 흔적은 보존.
        if not self.autonomy_enabled:
            if situation == "C":
                self.recheck_obstacle_on_enable = True
            return

        # --------------------------------------------------------------
        # 빨간 신호등
        # --------------------------------------------------------------

        if situation == "RED":

            # 일반 수동 주행 중일 때만 적용
            if self.state == DrivingState.MANUAL:

                # 현재 사용자가 전진 입력을 하고 있을 때 정지 latch
                if (
                    self.is_user_cmd_fresh()
                    and self.latest_user_cmd.linear.x > 0.01
                ):
                    self.traffic_stop_latched = True

                    self.get_logger().info(
                        "Traffic light RED - "
                        "forward command stopped until release"
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

        # BLOCKED에서 복구한 경우에만 전진키 해제 후 재입력을 요구.
        # 정상 회피 성공은 아래 플래그를 꺼서 바로 재출발 가능.
        if self.forward_rearm_required:
            if msg.linear.x <= 0.01:
                self.forward_release_seen = True
            elif (
                self.state == DrivingState.MANUAL
                and self.forward_release_seen
            ):
                self.forward_rearm_required = False
                self.forward_release_seen = False
                self.get_logger().info("Fresh forward input accepted")

        # ----------------------------------------------------------
        # 빨간불 정지 latch 해제
        #
        # RED에 의해 정지한 뒤 사용자가 전진 버튼을 놓으면
        # linear.x가 0이 되므로 latch를 해제한다.
        #
        # 이후 다시 전진 버튼을 누르면 정상 주행 가능.
        # ----------------------------------------------------------

        if (
            self.traffic_stop_latched
            and abs(msg.linear.x) <= 0.01
        ):
            self.traffic_stop_latched = False

            self.get_logger().info(
                "Forward command released - "
                "traffic stop latch cleared"
            )

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
        self.latest_scan = msg

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

        # OFF 동안 발생한 C가 다시 ON 시 재발행되지 않는 경우 복구.
        # 평상시 회피 시작 판단 및 0.44m 조건은 situation_node가 담당.
        if self.autonomy_enabled and self.recheck_obstacle_on_enable:
            if self.state != DrivingState.MANUAL:
                # 새로운 C가 먼저 정상 수신돼 회피를 시작한 상황.
                self.recheck_obstacle_on_enable = False
            elif (math.isfinite(self.front_distance)
                  and self.front_distance <= self.enable_recheck_stop_distance):
                self.recheck_obstacle_on_enable = False
                self.get_logger().info(
                    "Autonomy ON: obstacle still close; restarting avoidance"
                )
                self.c_avoidance_requested = True
                self.enter_stopped_state()
            elif (not math.isfinite(self.front_distance)
                  or self.front_distance >= self.enable_recheck_release_distance):
                # 장애물이 충분히 멀어져 situation_node도 다시 C 발행 가능.
                self.recheck_obstacle_on_enable = False

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
            self.goal_candidates = []
            self.goal_plan_queue = []
            self.goal_plan_index = 0
            self.goal_plan_results = []
            self.goal_plan_in_progress = False
            self.goal_plan_started_at = None
            self.goal_plan_generation += 1
            self.goal_request_in_progress = False
            self.goal_handle = None
            self.goal_cancel_requested = False

            self.replan_requested = False
            self.replan_count = 0
            self.pending_block_reason = None
            self.return_clear_since = None

            self.emergency_replan_armed = False
            self.nav_goal_accepted_time = None
            self.clear_condition_since = None
            self.emergency_condition_since = None

        self.change_state(DrivingState.STOPPED)

    # ==============================================================
    # Supervisor
    # ==============================================================

    def supervisor_callback(self) -> None:
        if not self.autonomy_enabled:
            return

        if self.state == DrivingState.STOPPED:
            self.handle_stopped_state()

        elif self.state == DrivingState.AVOIDING:
            self.handle_avoiding_state()

        elif self.state == DrivingState.REPLANNING:
            self.handle_replanning_state()

        elif self.state == DrivingState.BLOCKED:
            self.handle_blocked_recovery()

        elif self.state == DrivingState.VERIFYING_CLEAR:
            self.handle_verify_clear()

        elif (
            self.state == DrivingState.WAITING_FOR_NAV
            and self.goal_plan_in_progress
            and self.goal_plan_started_at is not None
            and self.get_age_seconds(self.goal_plan_started_at)
            > self.goal_plan_timeout
        ):
            self.enter_blocked_state("Side-goal planning timed out")

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

        # OFF 직전 전송된 Nav2 목표가 뒤늦게 승인될 수 있으므로,
        # 이전 세션의 목표 취소/종료가 정리된 뒤에 새 목표를 생성한다.
        if self.pending_nav_goal_accepts or self.detached_goal_keys:
            self.get_logger().warn(
                "Waiting for previous Nav2 goal cancellation...",
                throttle_duration_sec=2.0,
            )
            return

        if not (
            self.navigate_client.server_is_ready()
            and self.path_client.server_is_ready()
        ):
            self.get_logger().warn(
                "Waiting for /navigate_to_pose and /compute_path_to_pose...",
                throttle_duration_sec=2.0,
            )
            return

        self.start_goal_search()

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
    # 짧은 회피 목표 후보 생성 + Nav2 경로 검증
    # ============================================================== 

    def generate_side_goal_candidates(self) -> list[tuple[float, float]]:
        """최초 전방 장애물의 양옆에 짧은 임시 목표 후보를 만든다.

        반환 좌표는 odom 기준이다. 좌/우를 미리 선택하지 않으며
        실제 이동 가능 여부는 ComputePathToPose로 확인한다.
        """
        scan = self.latest_scan
        if scan is None or not self.is_scan_fresh():
            return []

        # LiDAR 스캔에서 인접 빔의 점들을 하나의 장애물로 묶는다.
        # (일반적인 박스/사람/벽 모두 동일한 처리)
        clusters = []
        cluster = []
        previous_index = None
        previous_point = None

        def append_cluster():
            if len(cluster) >= self.minimum_cluster_points:
                clusters.append(list(cluster))

        for i, distance in enumerate(scan.ranges):
            if not (
                math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max
            ):
                append_cluster()
                cluster = []
                previous_index = None
                previous_point = None
                continue

            angle = scan.angle_min + i * scan.angle_increment
            scan_x = distance * math.cos(angle)
            scan_y = distance * math.sin(angle)

            if scan_x <= 0.0 or scan_x > self.max_detection_distance:
                append_cluster()
                cluster = []
                previous_index = None
                previous_point = None
                continue

            point = (self.lidar_offset_x + scan_x, scan_y)
            if previous_point is not None and previous_index == i - 1:
                separation = math.hypot(
                    point[0] - previous_point[0],
                    point[1] - previous_point[1],
                )
                if separation > 0.18:
                    append_cluster()
                    cluster = []
            else:
                append_cluster()
                cluster = []

            cluster.append(point)
            previous_point = point
            previous_index = i

        append_cluster()

        # 정면 통로와 실제로 겹치는 장애물 중 가장 가까운 하나.
        front_clusters = [
            c for c in clusters
            if any(abs(y) <= self.corridor_half_width for _, y in c)
        ]
        if not front_clusters:
            return []

        obstacle = min(
            front_clusters,
            key=lambda c: min(
                x for x, y in c
                if abs(y) <= self.corridor_half_width
            ),
        )
        near_x = min(x for x, _ in obstacle)
        left_edge = max(y for _, y in obstacle)
        right_edge = min(y for _, y in obstacle)

        # LiDAR로 본 물체 가장자리에 차체 반폭과 여유거리를 추가.
        # 모델의 max Y footprint=0.39m와 goal tolerance=0.40m 고려.
        left_y = left_edge + self.robot_half_width + self.goal_clearance
        right_y = right_edge - self.robot_half_width - self.goal_clearance

        # 아주 짧은 측면 이동만 허용. 벽처럼 큰 장애물은 회피하지 않음.
        local_candidates = []
        for side in (+1, -1):
            base_y = left_y if side > 0 else right_y
            for extra_side in self.goal_side_variants:
                target_y = base_y + side * extra_side
                if abs(target_y) > self.max_goal_side_offset:
                    continue
                for extra_x in self.goal_x_variants:
                    target_x = max(0.90, near_x + extra_x)
                    if target_x > self.max_goal_forward:
                        continue
                    local_candidates.append((target_x, target_y))

        # odom 위치로 변환. 회피 방향은 현재 차체 자세 기준.
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        goals = []
        for x, y in local_candidates:
            world_x = self.current_x + cos_yaw * x - sin_yaw * y
            world_y = self.current_y + sin_yaw * x + cos_yaw * y
            goals.append((world_x, world_y))

        self.get_logger().info(
            f"Short avoidance goals: obstacle_x={near_x:.2f}, "
            f"edges=({right_edge:.2f}, {left_edge:.2f}), "
            f"candidates={len(goals)}"
        )
        return goals

    def make_nav_pose(self, x: float, y: float):
        """원래 주행 방향을 유지하는 odom 목표 PoseStamped."""
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        half_yaw = self.original_travel_yaw * 0.5
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def start_goal_search(self) -> None:
        """후보를 계획 가능한지 검증하는 동안 휠체어는 정지."""
        goals = self.generate_side_goal_candidates()
        if not goals:
            self.enter_blocked_state(
                "No close side-goal candidate from LiDAR"
            )
            return

        self.goal_plan_generation += 1
        self.goal_plan_queue = goals
        self.goal_plan_index = 0
        self.goal_plan_results = []
        self.goal_plan_in_progress = True
        self.goal_plan_started_at = self.get_clock().now()
        self.change_state(DrivingState.WAITING_FOR_NAV)
        self.check_next_goal_path(self.goal_plan_generation)

    def check_next_goal_path(self, generation: int) -> None:
        if (
            generation != self.goal_plan_generation
            or self.state != DrivingState.WAITING_FOR_NAV
        ):
            return

        if self.goal_plan_index >= len(self.goal_plan_queue):
            self.goal_plan_in_progress = False
            self.goal_plan_started_at = None
            if not self.goal_plan_results:
                self.enter_blocked_state(
                    "Nav2 found no short feasible side-goal path"
                )
                return

            # 기존 ±50도 목표 우선순위는 유지.
            # 선호 범위 안의 유효 후보끼리만 측면 이탈 가중치를 반영한다.
            # 선호 범위에 유효 후보가 없거나 모두 실패했다면
            # 범위 밖 후보는 종전처럼 경로 길이 순으로 시도한다.
            cos_yaw = math.cos(self.original_travel_yaw)
            sin_yaw = math.sin(self.original_travel_yaw)
            angle_limit = math.radians(self.preferred_goal_angle_deg)

            def goal_heading_angle(candidate):
                _, goal_x, goal_y, _ = candidate
                dx = goal_x - self.goal_start_x
                dy = goal_y - self.goal_start_y
                forward = cos_yaw * dx + sin_yaw * dy
                side = -sin_yaw * dx + cos_yaw * dy
                return math.atan2(side, forward)

            def goal_sort_key(candidate):
                path_length, goal_x, goal_y, max_side = candidate
                preferred = abs(goal_heading_angle(candidate)) <= angle_limit
                # 선호 범위 안에서만 측면 이탈을 줄이도록 작은 가중치 부여.
                # 범위 밖은 기존 최단 경로 순서를 그대로 유지한다.
                score = (
                    path_length + self.lateral_deviation_weight * max_side
                    if preferred else path_length
                )
                return (0 if preferred else 1, score, path_length, goal_x, goal_y)

            self.goal_candidates = sorted(
                self.goal_plan_results,
                key=goal_sort_key,
            )
            preferred_count = sum(
                abs(goal_heading_angle(candidate)) <= angle_limit
                for candidate in self.goal_candidates
            )
            self.goal_candidate_index = 0
            chosen = self.goal_candidates[0]
            chosen_angle = math.degrees(goal_heading_angle(chosen))
            chosen_preferred = abs(goal_heading_angle(chosen)) <= angle_limit
            chosen_score = (
                chosen[0] + self.lateral_deviation_weight * chosen[3]
                if chosen_preferred else chosen[0]
            )
            self.get_logger().info(
                f"Selected short goal from {len(self.goal_candidates)} "
                f"valid paths (within ±{self.preferred_goal_angle_deg:.0f}°: "
                f"{preferred_count}); "
                f"goal_angle={chosen_angle:.1f}°, "
                f"path={chosen[0]:.2f}m, max_side={chosen[3]:.2f}m, "
                f"score={chosen_score:.2f}, "
                f"goal=({chosen[1]:.2f}, {chosen[2]:.2f})"
            )
            self.send_current_candidate_goal()
            return

        x, y = self.goal_plan_queue[self.goal_plan_index]
        goal = ComputePathToPose.Goal()
        goal.goal = self.make_nav_pose(x, y)
        goal.planner_id = "GridBased"
        goal.use_start = False

        future = self.path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self.goal_path_response_callback(f, generation)
        )

    def goal_path_response_callback(self, future, generation: int) -> None:
        if generation != self.goal_plan_generation:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError("planner rejected candidate")
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.goal_path_result_callback(f, generation)
            )
        except Exception as exc:
            self.get_logger().warn(f"Goal path request failed: {exc}")
            self.goal_plan_index += 1
            self.check_next_goal_path(generation)

    def goal_path_result_callback(self, future, generation: int) -> None:
        if (
            generation != self.goal_plan_generation
            or self.state != DrivingState.WAITING_FOR_NAV
        ):
            return
        try:
            wrapped = future.result()
            path = wrapped.result.path.poses
            x, y = self.goal_plan_queue[self.goal_plan_index]

            if wrapped.status == GoalStatus.STATUS_SUCCEEDED and len(path) >= 2:
                length = sum(
                    math.hypot(
                        b.pose.position.x - a.pose.position.x,
                        b.pose.position.y - a.pose.position.y,
                    )
                    for a, b in zip(path[:-1], path[1:])
                )
                endpoint_error = math.hypot(
                    path[-1].pose.position.x - x,
                    path[-1].pose.position.y - y,
                )
                # 선택 경로가 너무 멀리 돌아가면 '짧은 회피'가 아님.
                yaw = self.original_travel_yaw
                cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
                max_side = max(
                    abs(
                        -sin_yaw * (p.pose.position.x - self.goal_start_x)
                        + cos_yaw * (p.pose.position.y - self.goal_start_y)
                    )
                    for p in path
                )

                if (
                    length <= self.max_goal_path_length
                    and max_side <= self.max_goal_path_side
                    and endpoint_error <= 0.25
                ):
                    self.goal_plan_results.append((length, x, y, max_side))
        except Exception as exc:
            self.get_logger().warn(f"Candidate path failed: {exc}")

        self.goal_plan_index += 1
        self.check_next_goal_path(generation)

    # ============================================================== 
    # Nav2 목표 전송
    # ============================================================== 

    def send_current_candidate_goal(self) -> None:
        if self.goal_candidate_index >= len(self.goal_candidates):
            self.enter_blocked_state("No feasible side-goal remains")
            return

        path_length, goal_x, goal_y, path_max_side = self.goal_candidates[
            self.goal_candidate_index
        ]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_nav_pose(goal_x, goal_y)

        self.goal_request_in_progress = True
        self.change_state(DrivingState.WAITING_FOR_NAV)
        self.get_logger().info(
            f"Sending side-goal {self.goal_candidate_index + 1}/"
            f"{len(self.goal_candidates)}: "
            f"goal=({goal_x:.2f}, {goal_y:.2f}), "
            f"path_length={path_length:.2f}m, "
            f"max_side={path_max_side:.2f}m"
        )

        epoch = self.control_epoch
        self.pending_nav_goal_accepts += 1
        try:
            future = self.navigate_client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda f, e=epoch: self.goal_response_callback(f, e)
            )
        except Exception as exc:
            self.pending_nav_goal_accepts -= 1
            self.goal_request_in_progress = False
            self.get_logger().error(f"Failed to dispatch Nav2 goal: {exc}")
            self.try_next_goal_candidate()

    def goal_response_callback(self, future, epoch: int) -> None:
        self.pending_nav_goal_accepts = max(0, self.pending_nav_goal_accepts - 1)

        # 예전 ON 세션에서 보낸 목표가 OFF 뒤에 승인된 경우 즉시 취소.
        # 결과가 이전 또는 현재 MANUAL 상태에 개입해서는 안 된다.
        if epoch != self.control_epoch or not self.autonomy_enabled:
            try:
                stale_handle = future.result()
                if stale_handle is not None and stale_handle.accepted:
                    self.cancel_detached_goal(stale_handle)
            except Exception as exc:
                self.get_logger().warn(f"Old Nav2 goal response ignored: {exc}")
            return

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

        self.change_state(
            DrivingState.AVOIDING
        )

        self.get_logger().info(
            "Avoidance goal accepted by Nav2"
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            lambda f, e=epoch: self.navigation_result_callback(f, e)
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

        if (
            self.replan_count
            >= self.maximum_replan_count
        ):

            self.cancel_for_blocked_state(
                "Maximum automatic replan count reached"
            )

            return

        self.get_logger().warn(
            "Confirmed new obstacle during avoidance. "
            "Canceling the current goal and replanning."
        )

        self.publish_stop()
        self.change_state(DrivingState.WAITING_FOR_NAV)

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

        cancel_future = (
            self.goal_handle.cancel_goal_async()
        )

        epoch = self.control_epoch
        cancel_future.add_done_callback(
            lambda f, e=epoch: self.cancel_response_callback(f, e)
        )

    def cancel_for_blocked_state(
        self,
        reason: str,
    ) -> None:

        self.replan_requested = False
        self.pending_block_reason = reason
        self.change_state(DrivingState.WAITING_FOR_NAV)
        self.publish_stop()

        if self.goal_handle is None:

            self.enter_blocked_state(reason)

            return

        self.get_logger().error(reason)

        self.cancel_current_goal()

    def cancel_response_callback(self, future, epoch: int) -> None:
        if epoch != self.control_epoch or not self.autonomy_enabled:
            return

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

    def navigation_result_callback(self, future, epoch: int) -> None:
        if epoch != self.control_epoch or not self.autonomy_enabled:
            return

        # 이미 다른 사유로 BLOCKED가 확정된 뒤 도착한 예전 결과는 무시.
        if self.state == DrivingState.BLOCKED:
            return

        try:

            wrapped_result = future.result()
            status = wrapped_result.status

        except Exception as error:

            self.get_logger().error(
                f"Failed to receive Nav2 result: {error}"
            )

            self.goal_handle = None
            self.goal_request_in_progress = False

            if self.pending_block_reason is not None:
                reason = self.pending_block_reason
                self.pending_block_reason = None
                self.enter_blocked_state(reason)
                return

            if self.replan_requested:

                self.change_state(
                    DrivingState.REPLANNING
                )

            else:
                self.try_next_goal_candidate()

            return

        self.goal_handle = None
        self.goal_request_in_progress = False

        if self.pending_block_reason is not None:
            reason = self.pending_block_reason
            self.pending_block_reason = None
            self.enter_blocked_state(reason)
            return

        if status == GoalStatus.STATUS_SUCCEEDED and self.goal_cancel_requested:
            # 취소를 요청했는데 도착 결과가 먼저 나올 수 있다.
            self.goal_cancel_requested = False
            self.latest_nav_cmd = Twist()
            self.last_nav_cmd_time = None
            if self.replan_requested:
                self.change_state(DrivingState.REPLANNING)
            else:
                self.enter_blocked_state("Nav2 finished while cancellation was pending")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:

            self.handle_navigation_success()

            return

        if self.goal_cancel_requested:

            self.goal_cancel_requested = False

            self.latest_nav_cmd = Twist()
            self.last_nav_cmd_time = None

            if self.replan_requested:

                self.change_state(
                    DrivingState.REPLANNING
                )

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

        # 현재 위치 기준으로 목표를 다시 생성한다.
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

        self.start_goal_search()

    def try_next_goal_candidate(self) -> None:

        self.goal_handle = None
        self.goal_request_in_progress = False

        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None

        self.goal_candidate_index += 1

        if (
            self.goal_candidate_index
            >= len(self.goal_candidates)
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
        # Nav2의 0.40m 목표 허용 오차만으로는 회피 완료를
        # 보장할 수 없다. 경로가 끝나도 바로 전진권을 넘기지 않는다.
        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None
        # 정상 회피 성공과 BLOCKED 복구는 다르게 처리한다.
        # 성공했을 때는 전진키 재입력 잠금이 없어야 한다.
        self.forward_rearm_required = False
        self.forward_release_seen = False
        self.return_clear_since = None
        self.change_state(DrivingState.VERIFYING_CLEAR)
        self.get_logger().info(
            "Short goal reached: verifying front corridor before MANUAL"
        )

    def handle_verify_clear(self) -> None:
        if not self.is_scan_fresh() or not self.is_odom_fresh():
            self.return_clear_since = None
            return
        if (math.isfinite(self.raw_front_distance)
                and self.raw_front_distance < self.return_clear_distance):
            # 목표점에는 도착했지만 장애물과의 전방 간격이 부족하다.
            self.enter_blocked_state("Goal reached but front corridor is still blocked")
            return

        if self.return_clear_since is None:
            self.return_clear_since = self.get_clock().now()
            return
        if self.get_age_seconds(self.return_clear_since) < self.return_clear_duration:
            return

        self.publish_avoidance_status("COMPLETED")
        self._return_to_manual("Avoidance completed successfully")

    def _return_to_manual(self, message: str) -> None:
        self.latest_user_cmd = Twist()
        self.last_user_cmd_time = None
        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None
        self.c_avoidance_requested = False
        self.reset_avoidance_session()
        self.change_state(DrivingState.MANUAL)
        self.get_logger().info(message)

    def handle_blocked_recovery(self) -> None:
        # BLOCKED에서 자동 후진은 하지 않는다. 사용자가 직접
        # 후진/회전해 실제로 위험 지점에서 벗어났을 때만 수동 복귀.
        if not self.is_scan_fresh() or not self.is_odom_fresh():
            self.blocked_clear_since = None
            return

        if self.blocked_pose is None:
            # Odometry가 BLOCKED 진입 시 끊겼다가 복구된 경우
            # 그때의 위치부터 복구 움직임을 확인한다.
            self.blocked_pose = (self.current_x, self.current_y, self.current_yaw)
            return
        x0, y0, yaw0 = self.blocked_pose
        moved = math.hypot(self.current_x - x0, self.current_y - y0)
        dyaw = math.atan2(math.sin(self.current_yaw - yaw0),
                          math.cos(self.current_yaw - yaw0))
        escaped = (moved >= self.blocked_min_translation
                   or abs(dyaw) >= self.blocked_min_rotation)
        clear = (not math.isfinite(self.raw_front_distance)
                 or self.raw_front_distance >= self.blocked_clear_distance)
        if not escaped or not clear:
            self.blocked_clear_since = None
            return

        if self.blocked_clear_since is None:
            self.blocked_clear_since = self.get_clock().now()
            return
        if self.get_age_seconds(self.blocked_clear_since) < self.blocked_clear_duration:
            return

        # 전진 재시작은 사용자에게 맡긴다.
        self.publish_avoidance_status("RECOVERED")
        self._return_to_manual("BLOCKED recovered: MANUAL, fresh forward input required")

    def enter_blocked_state(
        self,
        reason: str,
    ) -> None:

        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = None

        self.goal_handle = None
        self.goal_request_in_progress = False
        self.goal_cancel_requested = False
        self.replan_requested = False
        self.pending_block_reason = None
        self.blocked_pose = (self.current_x, self.current_y, self.current_yaw) if self.is_odom_fresh() else None
        self.blocked_clear_since = None
        self.return_clear_since = None
        self.forward_rearm_required = True
        self.forward_release_seen = False
        self.latest_user_cmd = Twist()
        self.last_user_cmd_time = None

        # 대기 중이던 planner callback이 재진입하지 못하게 무효화.
        self.goal_plan_generation += 1
        self.goal_plan_in_progress = False
        self.goal_plan_started_at = None
        self.goal_plan_queue = []
        self.goal_candidates = []

        self.change_state(
            DrivingState.BLOCKED
        )

        self.get_logger().error(
            f"{reason}. "
            "Manual reverse and rotation are available."
        )

    def reset_avoidance_session(self) -> None:

        self.stopped_since = None
        self.avoidance_pose_saved = False

        self.goal_candidate_index = 0
        self.goal_candidates = []
        self.goal_plan_queue = []
        self.goal_plan_results = []
        self.goal_plan_index = 0
        self.goal_plan_generation += 1
        self.goal_plan_in_progress = False
        self.goal_plan_started_at = None
        self.goal_request_in_progress = False
        self.goal_handle = None
        self.goal_cancel_requested = False

        self.replan_requested = False
        self.replan_count = 0

        self.emergency_replan_armed = False
        self.nav_goal_accepted_time = None
        self.clear_condition_since = None
        self.emergency_condition_since = None
        self.blocked_pose = None
        self.blocked_clear_since = None
        self.return_clear_since = None

    # ==============================================================
    # 최종 속도 출력
    # ==============================================================

    def output_callback(self) -> None:
        # OFF 상태에서는 scan timeout, RED latch, BLOCKED, STOPPED 등
        # 모든 자동 주행 제한을 무시하고 Teleop을 모터에 그대로 중계.
        # Teleop 입력 타임아웃만 유지하여 오래된 명령이 계속 남지 않게 한다.
        if not self.autonomy_enabled:
            self.publish_manual_or_stop()
            return

        if not self.is_scan_fresh():

            self.publish_stop()

            return

        # ----------------------------------------------------------
        # 빨간불 횡단보도 정지
        #
        # RED를 받은 뒤 사용자가 기존 전진 버튼을 계속 누르고 있으면
        # 실제 cmd_vel에는 계속 0을 발행한다.
        #
        # 전진 버튼을 놓으면 user_cmd_callback에서 latch가 해제된다.
        # ----------------------------------------------------------

        if self.traffic_stop_latched:

            self.publish_stop()

            return

        if self.state == DrivingState.MANUAL:

            if not self.is_user_cmd_fresh() or (
                self.forward_rearm_required and self.latest_user_cmd.linear.x > 0.01
            ):
                self.publish_stop()
                return

            self.cmd_vel_publisher.publish(
                self.copy_twist(
                    self.latest_user_cmd
                )
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
                self.copy_twist(
                    self.latest_nav_cmd
                )
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

        # WAITING_FOR_NAV, REPLANNING, VERIFYING_CLEAR에서는 정지
        self.publish_stop()

    def make_recovery_cmd(
        self,
        user_cmd: Twist,
    ) -> Twist:
        """
        전진만 차단하고 후진과 회전은 허용한다.
        """

        output_cmd = self.copy_twist(
            user_cmd
        )

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
            self.get_age_seconds(
                self.last_scan_time
            )
            <= self.scan_timeout
        )

        if (
            not fresh
            and not self.scan_timed_out
        ):

            self.scan_timed_out = True

            self.get_logger().error(
                "LiDAR timeout: output is stopped"
            )

        return fresh

    def is_user_cmd_fresh(self) -> bool:

        if self.last_user_cmd_time is None:
            return False

        return (
            self.get_age_seconds(
                self.last_user_cmd_time
            )
            <= self.user_cmd_timeout
        )

    def is_nav_cmd_fresh(self) -> bool:

        if self.last_nav_cmd_time is None:
            return False

        return (
            self.get_age_seconds(
                self.last_nav_cmd_time
            )
            <= self.nav_cmd_timeout
        )

    def is_odom_fresh(self) -> bool:

        if self.last_odom_time is None:
            return False

        return (
            self.get_age_seconds(
                self.last_odom_time
            )
            <= self.odom_timeout
        )

    def get_age_seconds(
        self,
        recorded_time,
    ) -> float:

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

        if math.isinf(
            self.front_distance
        ):
            distance_text = "CLEAR"

        else:
            distance_text = (
                f"{self.front_distance:.2f}m"
            )

        if math.isinf(
            self.raw_front_distance
        ):
            raw_text = "CLEAR"

        else:
            raw_text = (
                f"{self.raw_front_distance:.2f}m"
            )

        self.get_logger().info(
            f"autonomy={'ON' if self.autonomy_enabled else 'OFF'}, "
            f"state={self.state.value}, "
            f"front_distance={distance_text}, "
            f"raw={raw_text}, "
            f"pose=({self.current_x:.2f}, "
            f"{self.current_y:.2f}, "
            f"yaw={self.current_yaw:.2f})"
        )

        self.last_logged_state = self.state
        self.last_logged_distance = (
            self.front_distance
        )

    def should_log_status(self) -> bool:

        if (
            self.last_logged_state
            != self.state
        ):
            return True

        if self.last_logged_distance is None:
            return True

        if math.isinf(
            self.front_distance
        ):

            return not math.isinf(
                self.last_logged_distance
            )

        if math.isinf(
            self.last_logged_distance
        ):
            return True

        return abs(
            self.front_distance
            - self.last_logged_distance
        ) >= 0.10

    def publish_avoidance_status(
        self,
        status: str,
    ) -> None:

        msg = String()
        msg.data = status

        self.avoidance_status_publisher.publish(
            msg
        )

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
            DrivingState.VERIFYING_CLEAR,
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
            f"{previous_state.value} -> "
            f"{new_state.value}"
        )

    # ==============================================================
    # 공통 함수
    # ==============================================================

    def publish_stop(self) -> None:

        if not rclpy.ok():
            return

        self.cmd_vel_publisher.publish(
            Twist()
        )

    @staticmethod
    def copy_twist(
        source: Twist,
    ) -> Twist:

        copied = Twist()

        copied.linear.x = source.linear.x
        copied.linear.y = source.linear.y
        copied.linear.z = source.linear.z

        copied.angular.x = source.angular.x
        copied.angular.y = source.angular.y
        copied.angular.z = source.angular.z

        return copied


def main(args=None) -> None:

    rclpy.init(
        args=args
    )

    node = NavAvoidanceNode()

    try:
        rclpy.spin(
            node
        )

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
