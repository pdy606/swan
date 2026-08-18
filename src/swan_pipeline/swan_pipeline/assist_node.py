#!/usr/bin/env python3
"""
[어시스트 노드] 부분 자율주행 상태머신 — 팀 합의 구조 반영
  수동주행 → (d_decel 이내) 감속개입 → (d_takeover 이내) 자율회피 → 제어권 반환 → 수동주행

입력:  /cmd_vel_user   (사용자 조작: teleop 또는 virtual_user)
       /swan/detections (인지: LiDAR/카메라/GT 무관 — 계약만 지키면 됨)
       /odom
출력:  /swan/drive_target → control_node(Jerk 제한) → /cmd_vel

[통합 포인트]
  - AUTO_AVOID 내부의 반응형 회피는 나중에 Nav2(가상 목적지 기반)로 교체 가능.
    상태머신·감속개입·제어권반환 골격은 그대로 유지됨.
  - 임계값은 전부 파라미터. 실험(param_sweep.sh)으로 결정:
    d_decel=2.0/d_take=0.8 은 표면여유 0.09m로 스침 → 3.0/1.5 로 상향.
    d_takeover=1.5 근거: 실측 S-curve 정지거리 1.28m 보다 커야 최악시 정지 가능.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from swan_interfaces.msg import DetectionArray, DriveTarget

MANUAL, ASSIST_DECEL, AUTO_AVOID, RETURN_PATH, RETURN, YIELD = 0, 1, 2, 3, 4, 5
NAMES = {MANUAL: '수동주행', ASSIST_DECEL: '감속개입', AUTO_AVOID: '자율회피',
         RETURN_PATH: '경로복귀', RETURN: '제어권반환', YIELD: '양보정지'}

# ── 클래스별 행동 규약 ──  (핵심: 전부 회피 X → 상황 판단)
#   avoid  = 조향 회피 (정적 물체)
#   yield  = 감속/정지 양보 (사람 등 동적 — 조향하면 상대 진로 침범 위험)
#   ignore = 무시하고 통과 (연석램프·횡단보도 등 주행면)
CLASS_BEHAVIOR = {
    0: 'avoid',    # kickboard
    1: 'avoid',    # car (불법주차)
    2: 'avoid',    # curb (연석 턱)
    3: 'yield',    # person (사람) ★
    4: 'ignore',   # tactile_paving (점자블록)
    5: 'avoid',    # pole/기타
    10: 'ignore',  # curb_ramp (건너기용 경사로) ★
    11: 'ignore',  # crosswalk
    12: 'signal',  # traffic_light (빨강=정지/초록=진행) — 물리장애물 아님
    99: 'avoid',   # unknown (LiDAR만) → 안전하게 회피 기본값
}


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def yaw_from_quat(z, w):
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def norm_angle(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class AssistNode(Node):
    def __init__(self):
        super().__init__('assist_node')
        # ※ 거리는 인지가 주는 '표면까지 여유거리(clearance)' 기준
        self.declare_parameter('d_decel', 3.0)     # 실험: 2.0은 여유 0.09m로 스침 → 3.0
        self.declare_parameter('d_takeover', 1.5)  # 근거: 실측 정지거리 1.28m 보다 커야 함
        self.declare_parameter('d_clear', 2.0)     # 해제 거리도 함께 상향
        self.declare_parameter('front_angle', 0.9)   # 전방 판정 ±rad (인지 FOV와 일치시켜 상태튐 방지)
        self.declare_parameter('v_max', 1.67)        # 보도 법정 6km/h
        self.declare_parameter('avoid_v', 0.8)
        self.declare_parameter('avoid_omega', 0.8)     # 조향 각속도 '상한'
        # ── 비례 회피 (필요한 만큼만 조향해서 자연스럽게) ──
        # 고정 각속도로 무작정 돌면 필요 이상으로 크게 피해 부자연스러움.
        # 목표: 장애물 옆을 safe_lateral 만큼 여유 두고 '스쳐 지나가기'
        self.declare_parameter('safe_lateral', 0.96)   # 충돌한계 0.56(반폭0.31+박스0.25) + 안전여유 0.40
        self.declare_parameter('avoid_kp', 2.0)        # 방위각 부족분 → 조향
        self.declare_parameter('geom_offset', 0.66)    # clearance → 중심거리 보정(반길이0.41+반폭0.25)
        self.declare_parameter('return_hold', 1.0)   # 반환 상태 유지 시간(s)
        self.declare_parameter('lost_hold', 0.7)     # 장애물 놓쳐도 이 시간은 유지(히스테리시스)
        self.declare_parameter('d_stop', 1.2)        # 양보: 사람 앞 이 거리에서 정지
        # ── 팀 통합 모드 (cmd_vel arbiter = gyuwon nav_avoidance_node) ──
        #   회피가 필요한 상황이면 스스로 조향하지 않고 /driving_situation="C" 를 발행해
        #   Nav2 회피를 gyuwon 노드에 위임하고, /avoidance_status="COMPLETED" 를 기다린다.
        self.declare_parameter('team_mode', False)
        self.declare_parameter('situation_topic', '/driving_situation')
        self.declare_parameter('avoidance_status_topic', '/avoidance_status')

        g = lambda n: self.get_parameter(n).value
        self.d_decel, self.d_take, self.d_clear = g('d_decel'), g('d_takeover'), g('d_clear')
        self.front_angle, self.v_max = g('front_angle'), g('v_max')
        self.avoid_v, self.avoid_omega = g('avoid_v'), g('avoid_omega')
        self.safe_lat, self.avoid_kp = g('safe_lateral'), g('avoid_kp')
        self.geom_off = g('geom_offset')
        self.return_hold, self.lost_hold = g('return_hold'), g('lost_hold')
        self.d_stop = g('d_stop')
        self.last_seen = None      # 마지막으로 장애물을 본 시각
        self.avoid_dir = 0.0       # 회피 방향 고정(중간에 안 바뀌게)
        self.obs_beh = 'avoid'     # 현재 장애물의 행동 분류

        self.declare_parameter('xtrack_kp', 0.8)     # 경로복귀: 횡오차 → 목표방향
        self.declare_parameter('yaw_kp', 1.5)        # 경로복귀: 방향오차 → 각속도
        # 복귀 완료 판정 (빡빡해야 함 — 각도 오차가 남으면 그대로 계속 밀려남)
        self.declare_parameter('ret_xt_tol', 0.12)   # 횡오차 허용 [m]
        self.declare_parameter('ret_yaw_tol', 0.03)  # 방향오차 허용 [rad] ≈ 1.7도
        self.declare_parameter('ret_timeout', 8.0)   # 복귀 최대 시간 [s] (무한루프 방지)
        self.xtrack_kp, self.yaw_kp = g('xtrack_kp'), g('yaw_kp')
        self.ret_xt_tol, self.ret_yaw_tol = g('ret_xt_tol'), g('ret_yaw_tol')
        self.ret_timeout = g('ret_timeout')
        self.rp_t = None                             # 경로복귀 시작 시각
        self.red_light = False                       # 신호등 빨강(카메라)
        self.red_seen = None
        self.obs_beh = 'avoid'

        self.team_mode = g('team_mode')
        self.nav_active = False       # 팀모드: 지금 Nav2 회피 위임 중인가

        self.create_subscription(Twist, '/cmd_vel_user', self.on_user, 10)
        self.create_subscription(DetectionArray, '/swan/detections', self.on_det, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.pub = self.create_publisher(DriveTarget, '/swan/drive_target', 10)
        self.state_pub = self.create_publisher(String, '/swan/state', 10)
        # 팀 통합 계약: 상황 발행 + 회피완료 수신
        self.sit_pub = self.create_publisher(String, g('situation_topic'), 10)
        if self.team_mode:
            self.create_subscription(String, g('avoidance_status_topic'),
                                     self.on_avoid_status, 10)
            self.get_logger().info(
                '팀 통합 모드: 회피는 /driving_situation="C" 로 Nav2(gyuwon) 위임, '
                '/avoidance_status 대기 (cmd_vel arbiter=gyuwon)')

        self.user_v = self.user_w = 0.0
        self.obs_d, self.obs_a, self.has_obs = 99.0, 0.0, False
        self.rx = self.ry = self.ryaw = 0.0
        self.path_y = self.path_yaw = 0.0   # 개입 직전의 '원래 경로' 기억
        self.state = MANUAL
        self.ret_t = None
        self.create_timer(0.05, self.loop)   # 20 Hz
        self.get_logger().info(
            f'assist 시작: 감속개입 {self.d_decel}m / 자율전환 {self.d_take}m / 해제 {self.d_clear}m')

    def on_user(self, m: Twist):
        self.user_v, self.user_w = m.linear.x, m.angular.z

    def on_odom(self, m: Odometry):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.rx, self.ry, self.ryaw = p.x, p.y, yaw_from_quat(q.z, q.w)
        if self.state == MANUAL:
            # 수동 주행 중에는 현재 진행선을 계속 '원래 경로'로 갱신
            self.path_y, self.path_yaw = self.ry, self.ryaw

    def on_det(self, msg: DetectionArray):
        best = None
        red = False
        for d in msg.detections:
            beh = CLASS_BEHAVIOR.get(d.class_id, 'avoid')
            if beh == 'signal':                          # 신호등: 장애물 아님, 별도 처리
                if 'red' in d.class_name:
                    red = True
                continue
            if beh == 'ignore':                          # 램프·횡단보도 등 = 반응 안 함
                continue
            if abs(d.angle_rad) < self.front_angle:      # 전방 콘 안에 있는 것만
                if best is None or d.distance_m < best.distance_m:
                    best = d
        # 신호등 상태 갱신 (빨강 보이면 정지)
        self.red_light = red
        if red:
            self.red_seen = self.get_clock().now()
        if best is None:
            # 히스테리시스: 잠깐 놓쳐도 lost_hold 동안은 '있다'고 유지 (상태 튐 방지)
            if self.last_seen is not None:
                dt = (self.get_clock().now() - self.last_seen).nanoseconds / 1e9
                if dt < self.lost_hold:
                    return
            self.has_obs = False
            self.obs_d = 99.0
        else:
            self.has_obs = True
            self.obs_d, self.obs_a = best.distance_m, best.angle_rad
            self.obs_beh = CLASS_BEHAVIOR.get(best.class_id, 'avoid')
            self.last_seen = self.get_clock().now()

    def path_error(self):
        """원래 경로선(path_y, path_yaw) 기준 횡오차·방향오차"""
        dx, dy = self.rx - 0.0, self.ry - self.path_y
        xtrack = -math.sin(self.path_yaw) * dx + math.cos(self.path_yaw) * dy
        yaw_err = norm_angle(self.ryaw - self.path_yaw)
        return xtrack, yaw_err

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'상태 전환: {NAMES[self.state]} → {NAMES[s]}  (장애물 {self.obs_d:.2f}m)')
            self.state = s
            if s == RETURN:
                self.ret_t = self.get_clock().now()
            if s == RETURN_PATH:
                self.rp_t = self.get_clock().now()
            if s in (MANUAL, RETURN):
                self.avoid_dir = 0.0      # 회피 끝나면 방향 고정 해제

    # ── 팀 통합 계약 핸들러 ──
    def on_avoid_status(self, m: String):
        if m.data.strip().upper() == 'COMPLETED' and self.nav_active:
            self.nav_active = False
            self.set_state(MANUAL)
            self.get_logger().info('avoidance COMPLETED 수신 → 정상주행 재개')

    def _publish_situation(self, letter):
        s = String(); s.data = letter; self.sit_pub.publish(s)

    def loop_team(self, d):
        """팀모드: 스스로 조향/제어하지 않고 상황만 판단해 발행.
        회피 필요(정적물체 근접) → 'C' 발행하고 Nav2(gyuwon) 위임, 완료까지 대기."""
        need_avoid = self.has_obs and self.obs_beh == 'avoid' and d < self.d_take
        if need_avoid and not self.nav_active:
            self._publish_situation('C')          # gyuwon: 'C' 만 회피 트리거
            self.nav_active = True
            self.set_state(AUTO_AVOID)
            self.get_logger().info(f'C 상황 발행 → Nav2 회피 위임 (장애물 {d:.2f}m)')
        elif not self.nav_active:
            self._publish_situation('A')          # 정상(비회피) 상황
            self.set_state(MANUAL)
        s = String(); s.data = NAMES[self.state]; self.state_pub.publish(s)

    def loop(self):
        d = self.obs_d if self.has_obs else 99.0

        # ── 팀 통합 모드: cmd_vel 은 gyuwon 이 담당 → 상황만 발행 ──
        if self.team_mode:
            self.loop_team(d)
            return

        # ── 신호등 빨강 = 최우선 정지 (카메라) ──
        if self.red_light:
            out = DriveTarget()
            out.header.stamp = self.get_clock().now().to_msg()
            out.mode = DriveTarget.MODE_BRAKE
            out.v_target, out.omega_target, out.brake_level = 0.0, 0.0, 2
            self.pub.publish(out)
            s = String(); s.data = '신호정지'; self.state_pub.publish(s)
            return

        # ── 상태 전이 (★ 행동 분기: 사람=양보 / 물체=회피) ──
        if self.state in (MANUAL, ASSIST_DECEL, YIELD):
            if not self.has_obs:
                self.set_state(MANUAL)
            elif self.obs_beh == 'yield':          # 사람 → 감속·정지 양보 (조향 X)
                self.set_state(YIELD if d < self.d_decel else MANUAL)
            else:                                   # 물체 → 회피
                if d < self.d_take:
                    self.set_state(AUTO_AVOID)
                elif d < self.d_decel:
                    self.set_state(ASSIST_DECEL)
                else:
                    self.set_state(MANUAL)
        elif self.state == AUTO_AVOID:
            if (not self.has_obs) or d > self.d_clear:
                self.set_state(RETURN_PATH)     # 장애물 벗어남 → 원래 경로로 복귀
        elif self.state == RETURN_PATH:
            if self.has_obs and d < self.d_take:
                self.set_state(AUTO_AVOID)      # 복귀 중 또 만나면 다시 회피
            else:
                xt, ye = self.path_error()
                # ※ 허용오차가 크면 '덜 정렬된 채' 손을 놓아 그 각도로 계속 직진 →
                #    90m 가는 동안 옆으로 12m 밀리는 문제가 생김. 각도는 특히 빡빡하게.
                rp_dt = (self.get_clock().now() - self.rp_t).nanoseconds / 1e9 if self.rp_t else 0.0
                if abs(xt) < self.ret_xt_tol and abs(ye) < self.ret_yaw_tol:
                    self.set_state(RETURN)      # 원래 경로에 복귀 완료
                elif rp_dt > self.ret_timeout:
                    self.get_logger().warn(f'경로복귀 타임아웃 ({rp_dt:.1f}s) — 횡오차 {xt:.2f}m, 방향 {ye:.3f}rad')
                    self.set_state(RETURN)
        elif self.state == RETURN:
            dt = (self.get_clock().now() - self.ret_t).nanoseconds / 1e9
            if dt > self.return_hold:
                self.set_state(MANUAL)

        # ── 상태별 출력 ──
        out = DriveTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        uv = clamp(self.user_v, 0.0, self.v_max)

        if self.state == MANUAL:
            out.mode, out.v_target, out.omega_target, out.brake_level = \
                DriveTarget.MODE_NORMAL, uv, self.user_w, 0
        elif self.state == ASSIST_DECEL:
            # 거리에 비례해 허용속도 축소. 단 0이 아니라 avoid_v 까지만 —
            # 완전히 멈추면 회피할 운동량이 없어 장애물 앞에 갇힌다.
            ratio = clamp((d - self.d_take) / max(0.01, self.d_decel - self.d_take), 0.0, 1.0)
            v_allow = self.avoid_v + (self.v_max - self.avoid_v) * ratio
            out.mode, out.v_target, out.omega_target, out.brake_level = \
                DriveTarget.MODE_DECEL, min(uv, v_allow), self.user_w, 1
        elif self.state == AUTO_AVOID:
            # 회피방향은 진입 시점에 고정 (중간에 좌우로 흔들리지 않게)
            if self.avoid_dir == 0.0:
                self.avoid_dir = -1.0 if self.obs_a > 0 else 1.0
            # ── 비례 회피: '필요한 만큼만' 조향 ──
            # 장애물 옆을 safe_lat 여유로 지나가려면 방위각이 need 이상이어야 한다.
            # need 를 이미 확보했으면 더 안 돌린다 → 과회피 방지 (자연스러운 스침)
            center_d = max(d + self.geom_off, self.safe_lat)
            need = math.asin(clamp(self.safe_lat / center_d, -1.0, 1.0))
            err = need - abs(self.obs_a)          # 부족한 방위각
            mag = clamp(self.avoid_kp * err, 0.0, self.avoid_omega) if err > 0 else 0.0
            turn = self.avoid_dir * mag
            out.mode = DriveTarget.MODE_AVOID_R if self.avoid_dir < 0 else DriveTarget.MODE_AVOID_L
            out.v_target, out.omega_target, out.brake_level = self.avoid_v, turn, 1
        elif self.state == YIELD:
            # ★ 양보: 사람 앞에서 감속→정지 (조향은 사용자 것 유지 = 스스로 피하지 않음)
            #   d_stop 에서 0, d_decel 에서 v_max 로 선형 감속 → 사람 지나가면 자동 재개
            ratio = clamp((d - self.d_stop) / max(0.01, self.d_decel - self.d_stop), 0.0, 1.0)
            out.mode = DriveTarget.MODE_BRAKE if ratio < 0.05 else DriveTarget.MODE_DECEL
            out.v_target = min(uv, self.v_max * ratio)
            out.omega_target = self.user_w          # 조향 개입 안 함
            out.brake_level = 2
        elif self.state == RETURN_PATH:
            # 원래 경로선으로 복귀: 횡오차 → 목표방향 → 각속도
            xt, ye = self.path_error()
            aim = clamp(-self.xtrack_kp * xt, -0.7, 0.7)     # 경로 쪽으로 조준
            omega = clamp(self.yaw_kp * norm_angle(aim - ye), -self.avoid_omega, self.avoid_omega)
            out.mode = DriveTarget.MODE_NORMAL
            out.v_target, out.omega_target, out.brake_level = self.avoid_v, omega, 0
        else:  # RETURN (제어권 반환)
            out.mode, out.v_target, out.omega_target, out.brake_level = \
                DriveTarget.MODE_NORMAL, uv, self.user_w, 0

        out.ttc = d / max(0.1, self.v_max)
        self.pub.publish(out)
        s = String(); s.data = NAMES[self.state]
        self.state_pub.publish(s)


def main():
    rclpy.init()
    rclpy.spin(AssistNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
