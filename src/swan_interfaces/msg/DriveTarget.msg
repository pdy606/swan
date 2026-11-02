# 판단 결과 = 제어에게 주는 목표 (판단 → 제어)
std_msgs/Header header
uint8   MODE_NORMAL=0
uint8   MODE_DECEL=1
uint8   MODE_AVOID_L=2
uint8   MODE_AVOID_R=3
uint8   MODE_BRAKE=4
uint8   mode

float32 v_target           # 목표 선속도 (m/s)
float32 omega_target       # 목표 각속도 (rad/s), +좌회전 / -우회전
uint8   brake_level        # 0~3 다단 제동 강도
float32 ttc                # 충돌예상시간 (s), 참고용
