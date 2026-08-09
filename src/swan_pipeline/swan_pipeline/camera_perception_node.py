#!/usr/bin/env python3
"""
[카메라 인지 노드]  실제 Gazebo 카메라 영상(/image) → 객체 인식 → /swan/cam_detections
- 의존성: numpy 만 (cv_bridge·opencv·torch 불필요) — sensor_msgs/Image(rgb8) 를 직접 디코드
- 지금: 색·기하 기반 검출(YOLO 대역). detect() 함수만 YOLO로 바꾸면 그대로 동작.
- 출력: LiDAR 와 같은 Detection 계약 (class_id/name, bbox, distance_m, angle_rad)
  → 클래스는 카메라, 거리는 LiDAR 가 담당하는 '퓨전' 전제.

★ 카메라 파트 핵심 산출물:
  1) 실제 영상 처리 (image → 인식)
  2) 신호등 빨강/초록 상태 판단
  3) 단안 거리 추정(bbox 하단 기하)
  4) LiDAR 와 통일된 Detection 인터페이스
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from swan_interfaces.msg import Detection, DetectionArray

# 우리 월드 물체의 대표 '색상(Hue)' → 클래스 매핑
#  ★ RGB 거리 대신 HSV Hue 로 판별 → 조명/밝기 변화에 강인 (월드 바뀌어도 동작)
#  실제 YOLO 로 교체 시 이 테이블 대신 모델 출력 사용
#  hue 는 0~360°, 최소 채도(sat)·명도(val)로 배경(하늘·바닥) 제외
COLOR_CLASSES = [
    # name,        hue_lo, hue_hi,  class_id
    ('person',     14,  47,  3),    # 보행자(주황/살구색)
    ('kickboard',  70, 165,  0),    # 킥보드/방치물(초록)
    ('car',       180, 260,  1),    # 불법주차차(파랑)
]
MIN_SAT = 0.28   # 이보다 채도 낮으면 무채색(하늘·바닥·벽) → 무시
MIN_VAL = 0.18   # 너무 어두우면 그림자 → 무시

# ── 실제 YOLO(COCO 학습) 사용 시: COCO class id → 우리 내부 (class_id, name) ──
#   assist_node/fusion 이 쓰는 계약과 일치시킴. 킥보드는 COCO 에 없어 자전거/오토바이로 근사.
COCO2OURS = {
    0:  (3,  'person'),         # person
    2:  (1,  'car'),            # car
    5:  (1,  'car'),            # bus  → 차량류
    7:  (1,  'car'),            # truck→ 차량류
    1:  (0,  'kickboard'),      # bicycle   ≈ 킥보드(근사, 정밀하려면 커스텀 학습)
    3:  (0,  'kickboard'),      # motorcycle≈ 킥보드(근사)
    9:  (12, 'traffic_light'),  # traffic light
}


class CameraPerception(Node):
    def __init__(self):
        super().__init__('camera_perception_node')
        self.declare_parameter('image_topic', '/wheelchair/camera/image')
        self.declare_parameter('out_topic', '/swan/cam_detections')
        self.declare_parameter('hfov', 1.204)          # 카메라 수평 화각 [rad]
        self.declare_parameter('camera_height', 0.90)  # 지면 기준 카메라 높이 [m]
        self.declare_parameter('focal_px', 0.0)        # 0이면 hfov로 계산
        self.declare_parameter('min_blob', 400)        # 최소 픽셀 수(노이즈 제거)
        self.declare_parameter('detector', 'color')    # 'color'(시뮬용 HSV) | 'yolo'(실물용)
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.35)      # YOLO 신뢰도 임계값
        g = lambda n: self.get_parameter(n).value
        self.hfov = g('hfov'); self.cam_h = g('camera_height')
        self.min_blob = g('min_blob'); self.fx_param = g('focal_px')
        self.mode = g('detector'); self.yolo_conf = g('yolo_conf')

        self.model = None
        if self.mode == 'yolo':
            from ultralytics import YOLO                # 무거운 import 는 yolo 모드에서만
            self.model = YOLO(g('yolo_model'))          # 최초 실행 시 가중치 자동 다운로드
            self.get_logger().info(f"YOLO 로드: {g('yolo_model')} (conf≥{self.yolo_conf})")

        self.create_subscription(Image, g('image_topic'), self.on_image, 5)
        self.pub = self.create_publisher(DetectionArray, g('out_topic'), 10)
        self.get_logger().info(
            f"camera_perception 시작: /image → 인식(detector={self.mode})")

    # ---- 영상 디코드 (rgb8 → numpy, cv_bridge 없이) ----
    def decode(self, msg: Image):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return None
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            arr = arr[:, :, ::-1]
        return arr

    # ---- RGB→HSV (벡터화, cv 없이) ----
    @staticmethod
    def rgb_to_hsv(img):
        """img(H,W,3 uint8) → hue[0~360], sat[0~1], val[0~1]"""
        r = img[:, :, 0] / 255.0; g = img[:, :, 1] / 255.0; b = img[:, :, 2] / 255.0
        mx = np.maximum.reduce([r, g, b]); mn = np.minimum.reduce([r, g, b])
        diff = mx - mn
        val = mx
        sat = np.where(mx > 1e-6, diff / np.maximum(mx, 1e-6), 0.0)
        hue = np.zeros_like(mx)
        nz = diff > 1e-6
        # 최대 채널별 hue 공식
        rm = nz & (mx == r); gm = nz & (mx == g) & ~rm; bm = nz & (mx == b) & ~rm & ~gm
        hue[rm] = ((g[rm] - b[rm]) / diff[rm]) % 6
        hue[gm] = (b[gm] - r[gm]) / diff[gm] + 2
        hue[bm] = (r[bm] - g[bm]) / diff[bm] + 4
        return hue * 60.0, sat, val

    # ---- 검출 (여기만 YOLO 로 교체하면 됨) : HSV Hue 기반 ----
    def detect(self, img):
        """반환: [(class_id, class_name, x0,y0,x1,y1, conf), ...]"""
        H, W, _ = img.shape
        hue, sat, val = self.rgb_to_hsv(img)
        chroma = (sat > MIN_SAT) & (val > MIN_VAL)   # 유채색만(배경 제외)
        dets = []
        for name, hlo, hhi, cid in COLOR_CLASSES:
            mask = chroma & (hue >= hlo) & (hue < hhi)
            n = int(mask.sum())
            if n < self.min_blob:
                continue
            ys, xs = np.where(mask)
            x0, x1, y0, y1 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
            conf = min(1.0, n / (self.min_blob * 5))
            dets.append((cid, name, x0, y0, x1, y1, conf))
        return dets

    # ---- 실제 YOLO 검출 (COCO 학습 모델) ----
    #  color 검출과 동일한 계약 반환 → estimate()/퓨전 그대로 재사용
    def detect_yolo(self, img):
        """반환: [(class_id, class_name, x0,y0,x1,y1, conf), ...]"""
        res = self.model(img, verbose=False, conf=self.yolo_conf)[0]
        dets = []
        for b in res.boxes:
            coco = int(b.cls[0])
            if coco not in COCO2OURS:                 # 관심 클래스만
                continue
            our_id, name = COCO2OURS[coco]
            x0, y0, x1, y1 = b.xyxy[0].tolist()
            dets.append((our_id, name, int(x0), int(y0), int(x1), int(y1), float(b.conf[0])))
        return dets

    # ---- 신호등 상태 (상단 영역 빨강/초록 비율) ----
    def traffic_light(self, img):
        H, W, _ = img.shape
        top = img[0:H // 3, :, :]           # 신호등은 위쪽
        r = ((top[:, :, 0] > 150) & (top[:, :, 1] < 80) & (top[:, :, 2] < 80)).sum()
        gmask = ((top[:, :, 1] > 150) & (top[:, :, 0] < 80) & (top[:, :, 2] < 80)).sum()
        if max(r, gmask) < 150:
            return None
        return 'red' if r > gmask else 'green'

    # ---- 단안 거리·각도 추정 (bbox 하단 기하) ----
    def estimate(self, x0, y0, x1, y1, W, H):
        fx = self.fx_param if self.fx_param > 0 else (W / 2) / math.tan(self.hfov / 2)
        cx, cy = W / 2, H / 2
        u = (x0 + x1) / 2
        v_bottom = y1
        angle = -math.atan2(u - cx, fx)                 # +좌/-우
        # 지면평면 가정: 아래쪽일수록 가까움
        dv = max(1.0, v_bottom - cy)
        distance = self.cam_h * fx / dv
        return float(max(0.0, distance)), float(angle)

    def on_image(self, msg: Image):
        img = self.decode(msg)
        if img is None:
            return
        H, W, _ = img.shape
        out = DetectionArray()
        out.header = msg.header
        out.img_w, out.img_h = W, H

        raw = self.detect_yolo(img) if self.mode == 'yolo' else self.detect(img)
        for cid, name, x0, y0, x1, y1, conf in raw:
            dist, ang = self.estimate(x0, y0, x1, y1, W, H)
            d = Detection()
            d.class_id, d.class_name = cid, name
            d.confidence = float(conf)
            d.bbox_x, d.bbox_y = float(x0 / W), float(y0 / H)
            d.bbox_w, d.bbox_h = float((x1 - x0) / W), float((y1 - y0) / H)
            d.distance_m, d.angle_rad = dist, ang
            d.risk_level = 2 if dist < 2.0 else 1
            out.detections.append(d)

        # 신호등 상태 → 특수 Detection (class_id 10=traffic_light)
        tl = self.traffic_light(img)
        if tl:
            d = Detection()
            d.class_id, d.class_name = 12, f"traffic_light_{tl}"
            d.confidence = 0.9
            d.distance_m, d.angle_rad = 3.0, 0.0
            d.risk_level = 2 if tl == 'red' else 0
            out.detections.append(d)

        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CameraPerception())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
