# 장애물 하나에 대한 인지 결과 (인지 → 판단)
uint16  class_id            # 0:kickboard 1:car 2:curb 3:person 4:tactile_paving 5:other
string  class_name
float32 confidence          # 0~1
float32 bbox_x              # 정규화 좌표(0~1)
float32 bbox_y
float32 bbox_w
float32 bbox_h
float32 distance_m          # 추정 거리 (m)
float32 angle_rad           # 방위각 (+왼쪽 / -오른쪽)
uint8   risk_level          # 0:safe 1:caution 2:danger
