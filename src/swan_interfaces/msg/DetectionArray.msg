# 한 프레임의 인지 결과 전체
std_msgs/Header header      # stamp(시각), frame_id="camera_link"
uint16 img_w
uint16 img_h
Detection[] detections     # 장애물 없으면 빈 배열 (계속 발행)
