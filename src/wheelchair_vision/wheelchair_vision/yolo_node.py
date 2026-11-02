import json
import math
import os

import traceback

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO



class YoloDetectorNode(Node):
    """YOLO 객체 인식 + 하이브리드 거리 측정 노드.

    - 바닥에 닿는 객체: 카메라 기하로 만든 ground-plane(BEV) homography로 X/Z(m) 계산
    - 공중 객체(신호등): box height + horizon 편차(dy) 융합식으로 Z 계산
    - Gap: 객체 중심 간 대각선이 아니라, 바닥 객체의 bbox 하단 좌/우 경계를 BEV로 투영해
      비슷한 깊이에 있는 객체 사이의 실제 수평 free gap을 계산
    """

    def __init__(self):
        super().__init__('yolo_detector_node')

        self.subscription = self.create_subscription(
            Image, '/camera', self.image_callback, 1
        )
        self.image_pub = self.create_publisher(Image, '/yolo/image_raw', 10)
        self.label_pub = self.create_publisher(
            String, '/yolo/detected_objects', 10
        )

        self.bridge = CvBridge()

        package_share = get_package_share_directory('wheelchair_vision')
        model_path = os.path.join(
            package_share, 'wheelchair_vision', 'best.pt'
        )

        if os.path.exists(model_path):
            self.get_logger().info(f'Loading custom model: {model_path}')
            self.model = YOLO(model_path)
        else:
            self.get_logger().warn(
                'best.pt not found! Using default yolov8n.pt'
            )
            self.model = YOLO('yolov8n.pt')

        # -----------------------------
        # ROS parameters
        # -----------------------------
        self.declare_parameter('confidence_threshold', 0.20)
        self.declare_parameter('max_dist_m', 6.0)

        # 현재 wheelchair model.sdf 기준 카메라 기본값
        # camera_link pose: z=0.64, pitch=0.17 rad
        # horizontal_fov=1.047 rad (약 60 deg)
        self.declare_parameter('camera_height_m', 0.64)
        self.declare_parameter('camera_pitch_rad', 0.17)
        self.declare_parameter('camera_hfov_rad', 1.047)

        # BEV homography를 구성할 기준 사각형(미터 단위)
        self.declare_parameter('bev_near_m', 1.5)
        self.declare_parameter('bev_half_width_m', 0.8)

        # 공중 객체 거리식: Z = air_scale / (box_h + air_dy_weight * dy)
        self.declare_parameter('air_scale', 250.0)
        self.declare_parameter('air_dy_weight', 0.5)

        # 비슷한 깊이의 물체끼리만 free gap 계산
        self.declare_parameter('gap_depth_tolerance_m', 1.5)
        self.declare_parameter('debug_overlay', True)

        self.conf_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.max_dist_m = float(self.get_parameter('max_dist_m').value)
        self.camera_height_m = float(
            self.get_parameter('camera_height_m').value
        )
        self.camera_pitch_rad = float(
            self.get_parameter('camera_pitch_rad').value
        )
        self.camera_hfov_rad = float(
            self.get_parameter('camera_hfov_rad').value
        )
        self.bev_near_m = float(self.get_parameter('bev_near_m').value)
        self.bev_half_width_m = float(
            self.get_parameter('bev_half_width_m').value
        )
        self.air_scale = float(self.get_parameter('air_scale').value)
        self.air_dy_weight = float(
            self.get_parameter('air_dy_weight').value
        )
        self.gap_depth_tolerance_m = float(
            self.get_parameter('gap_depth_tolerance_m').value
        )
        self.debug_overlay = bool(self.get_parameter('debug_overlay').value)

        # 커스텀 best.pt에서 확인된 공중 객체 클래스
        self.airborne_classes = {
            'red light stop',
            'green light go',
        }

        self.bev_matrix = None
        self.image_shape = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.horizon_y = None

    # ------------------------------------------------------------------
    # Camera geometry / BEV
    # ------------------------------------------------------------------
    def project_ground_point_to_image(self, x_right_m, z_forward_m):
        """바닥 평면의 (X right, Z forward)를 이미지 픽셀로 투영."""
        pitch = self.camera_pitch_rad
        c = math.cos(pitch)
        s = math.sin(pitch)

        # base/world-like camera frame vector:
        # forward +X, left +Y, up +Z
        # 사용자 표기 X는 화면 오른쪽을 +로 사용하므로 base Y = -X
        base_vec = np.array(
            [z_forward_m, -x_right_m, -self.camera_height_m],
            dtype=np.float64,
        )

        # camera -> base rotation around Y
        rot_cam_to_base = np.array(
            [
                [c, 0.0, s],
                [0.0, 1.0, 0.0],
                [-s, 0.0, c],
            ],
            dtype=np.float64,
        )

        cam_vec = rot_cam_to_base.T @ base_vec
        forward, left, up = cam_vec

        if forward <= 1e-6:
            raise ValueError('Ground calibration point is behind camera')

        # image right = -camera-left, image down = -camera-up
        u = self.cx + self.fx * (-left / forward)
        v = self.cy + self.fy * (-up / forward)
        return [u, v]

    def init_camera_geometry(self, h, w):
        self.image_shape = (h, w)

        self.fx = w / (2.0 * math.tan(self.camera_hfov_rad / 2.0))
        # Gazebo camera는 square pixel을 사용하므로 fx == fy로 둠
        self.fy = self.fx
        self.cx = w / 2.0
        self.cy = h / 2.0

        # pitch가 +이면 카메라가 아래쪽을 보는 현재 SDF convention 기준
        self.horizon_y = self.cy - self.fy * math.tan(
            self.camera_pitch_rad
        )

        near_z = self.bev_near_m
        far_z = self.max_dist_m
        half_w = self.bev_half_width_m

        # 실제 바닥의 네 점을 카메라 영상에 투영한 뒤,
        # 목적 좌표를 (X[m], Z[m])로 두면 결과가 바로 metric BEV 좌표가 된다.
        ground_corners = [
            (-half_w, near_z),
            (half_w, near_z),
            (half_w, far_z),
            (-half_w, far_z),
        ]

        src_pts = np.float32(
            [
                self.project_ground_point_to_image(x, z)
                for x, z in ground_corners
            ]
        )
        dst_pts = np.float32(ground_corners)

        self.bev_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

        self.get_logger().info(
            'Camera/BEV initialized: '
            f'{w}x{h}, fx={self.fx:.1f}, '
            f'horizon_y={self.horizon_y:.1f}px, '
            f'camera_h={self.camera_height_m:.2f}m, '
            f'pitch={self.camera_pitch_rad:.3f}rad'
        )

    def image_point_to_ground(self, x_px, y_px):
        """이미지의 바닥 접점 픽셀을 metric BEV (X, Z)로 변환."""
        pt = np.array([[[x_px, y_px]]], dtype=np.float32)
        ground = cv2.perspectiveTransform(pt, self.bev_matrix)[0][0]
        return float(ground[0]), float(ground[1])

    # ------------------------------------------------------------------
    # Distance estimation
    # ------------------------------------------------------------------
    def estimate_ground_object(self, x1, y1, x2, y2):
        x_center = (x1 + x2) / 2.0
        y_bottom = y2

        # horizon 위쪽은 지면과 교차하지 않으므로 유효한 ground point가 아님
        if y_bottom <= self.horizon_y + 1.0:
            return None

        dist_x, dist_z = self.image_point_to_ground(x_center, y_bottom)

        # bbox 하단 양 끝도 BEV로 보내 실제 자유 공간 추정에 사용
        left_x, _ = self.image_point_to_ground(x1, y_bottom)
        right_x, _ = self.image_point_to_ground(x2, y_bottom)

        edge_left_x = min(left_x, right_x)
        edge_right_x = max(left_x, right_x)

        return {
            'x': dist_x,
            'z': dist_z,
            'edge_left_x': edge_left_x,
            'edge_right_x': edge_right_x,
            'method': 'BEV',
        }

    def estimate_air_object(self, x_center, y_bottom, box_h):
        # 실제 카메라 pitch에서 계산된 horizon 사용
        dy = max(0.0, self.horizon_y - y_bottom)
        composite_feature = box_h + (dy * self.air_dy_weight)

        dist_z = self.air_scale / max(composite_feature, 1.0)

        # 기존의 임의 1.5 배율 대신 카메라 FOV로 X 거리 계산
        x_normalized = (x_center - self.cx) / self.fx
        dist_x = dist_z * x_normalized

        return {
            'x': float(dist_x),
            'z': float(dist_z),
            'method': 'AIR',
        }
    
    

    # ------------------------------------------------------------------
    # ROS callback
    # ------------------------------------------------------------------
    def image_callback(self, msg):
        try:
            # desired_encoding='bgr8'이면 CvBridge가 이미 BGR로 변환해준다.
            # 기존 RGB2BGR 재변환은 R/B channel을 다시 뒤집을 수 있으므로 제거.
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )

            h, w = cv_image.shape[:2]
            if self.bev_matrix is None or self.image_shape != (h, w):
                self.init_camera_geometry(h, w)

            results = self.model(cv_image, conf=self.conf_threshold)
            annotated_frame = results[0].plot()

            detected_objects = []

            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    # 모델에 없는 클래스 ID가 나와도 오류 없이 처리
                    if isinstance(self.model.names, dict):
                        class_name = self.model.names.get(
                        cls_id, f"Unknown_{cls_id}"
                    )
                    else:
                        class_name = (
                            self.model.names[cls_id]
                            if 0 <= cls_id < len(self.model.names)
                            else f"Unknown_{cls_id}"
                        )
                    class_key = class_name.strip().lower()

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    x_center = (x1 + x2) / 2.0
                    y_center = (y1 + y2) / 2.0
                    y_bottom = y2
                    box_h = y2 - y1

                    # 신호등은 bbox가 지면에 닿지 않으므로 class로 강제 AIR 분기
                    if class_key in self.airborne_classes:
                        estimate = self.estimate_air_object(
                            x_center, y_bottom, box_h
                        )
                    else:
                        # 나머지 클래스는 바닥 접점 기반 BEV
                        estimate = self.estimate_ground_object(
                            x1, y1, x2, y2
                        )

                    if estimate is None:
                        continue

                    dist_x = estimate['x']
                    dist_z = estimate['z']

                    # 음수/NaN/보정 범위 밖 값 제거
                    if not np.isfinite(dist_x) or not np.isfinite(dist_z):
                        continue
                    if dist_z <= 0.0 or dist_z > self.max_dist_m:
                        continue

                    obj = {
                        'class': class_name,
                        'x': float(dist_x),
                        'z': float(dist_z),
                        'box': (
                            int(x1), int(y1), int(x2), int(y2)
                        ),
                        'center': (int(x_center), int(y_center)),
                        'method': estimate['method'],
                    }

                    if estimate['method'] == 'BEV':
                        obj['edge_left_x'] = estimate['edge_left_x']
                        obj['edge_right_x'] = estimate['edge_right_x']

                    detected_objects.append(obj)

            # 가까운 Z 순서로 ranking
            detected_objects.sort(key=lambda obj: obj['z'])

            if self.debug_overlay:
                horizon_y_int = int(round(self.horizon_y))
                if 0 <= horizon_y_int < h:
                    cv2.line(
                        annotated_frame,
                        (0, horizon_y_int),
                        (w - 1, horizon_y_int),
                        (0, 255, 255),
                        1,
                    )
                    cv2.putText(
                        annotated_frame,
                        'Ground horizon',
                        (5, max(15, horizon_y_int - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        (0, 255, 255),
                        1,
                    )

            for i, obj in enumerate(detected_objects):
                rank = i + 1
                x1, y1, _, _ = obj['box']

                label = (
                    f"#{rank} {obj['class']} "
                    f"Z:{obj['z']:.2f}m X:{obj['x']:+.2f}m "
                    f"[{obj['method']}]"
                )
                text_y = y1 + 25 if y1 < 20 else y1 - 10

                cv2.putText(
                    annotated_frame,
                    label,
                    (x1, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 0),
                    4,
                )
                cv2.putText(
                    annotated_frame,
                    label,
                    (x1, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )

            # ----------------------------------------------------------
            # Gap 개선
            # ----------------------------------------------------------
            # 기존: Z 순서로 이웃한 객체 중심 간 대각선 거리
            # 개선: 바닥 객체를 X 순으로 정렬하고, 비슷한 깊이에 있을 때
            #       bbox 하단의 실제 BEV 좌/우 edge 사이 free gap을 계산한다.
            ground_objects = [
                obj for obj in detected_objects
                if obj['method'] == 'BEV'
            ]
            ground_objects.sort(key=lambda obj: obj['x'])

            for i in range(len(ground_objects) - 1):
                left_obj = ground_objects[i]
                right_obj = ground_objects[i + 1]

                depth_diff = abs(left_obj['z'] - right_obj['z'])
                if depth_diff > self.gap_depth_tolerance_m:
                    continue

                gap_m = (
                    right_obj['edge_left_x']
                    - left_obj['edge_right_x']
                )
                gap_m = max(0.0, float(gap_m))

                if gap_m <= 0.0:
                    continue

                # 화면에서는 각 bbox의 안쪽 하단 edge를 연결
                lx1, ly1, lx2, ly2 = left_obj['box']
                rx1, ry1, rx2, ry2 = right_obj['box']
                pt_left = (lx2, ly2)
                pt_right = (rx1, ry2)

                cv2.line(
                    annotated_frame,
                    pt_left,
                    pt_right,
                    (255, 0, 255),
                    2,
                )

                mid_x = (pt_left[0] + pt_right[0]) // 2
                mid_y = (pt_left[1] + pt_right[1]) // 2
                gap_label = f'Free Gap: {gap_m:.2f}m'

                cv2.putText(
                    annotated_frame,
                    gap_label,
                    (mid_x, max(20, mid_y - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 0),
                    4,
                )
                cv2.putText(
                    annotated_frame,
                    gap_label,
                    (mid_x, max(20, mid_y - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 105, 180),
                    2,
                )

            # integration-final 형식 유지: class/x/z JSON
            publish_objects = []
            for obj in detected_objects:
                publish_objects.append(
                    {
                        'class': str(obj['class']),
                        'x': float(obj['x']),
                        'z': float(obj['z']),
                    }
                )

            msg_str = String()
            msg_str.data = json.dumps(
                publish_objects, ensure_ascii=False
            )
            self.label_pub.publish(msg_str)

            # YOLO 결과 이미지를 ROS Image 메시지로 직접 변환
            # cv_bridge.cv2_to_imgmsg()의 KeyError: 16 우회

            annotated_frame = np.ascontiguousarray(
                annotated_frame,
                dtype=np.uint8
            )

            annotated_msg = Image()

            annotated_msg.header = msg.header
            annotated_msg.height = int(annotated_frame.shape[0])
            annotated_msg.width = int(annotated_frame.shape[1])

            annotated_msg.encoding = 'bgr8'
            annotated_msg.is_bigendian = 0
            annotated_msg.step = int(annotated_frame.shape[1] * 3)

            annotated_msg.data = annotated_frame.tobytes()

            self.image_pub.publish(annotated_msg)

        except Exception:
            self.get_logger().error(
                "Error processing image:\n"
                + traceback.format_exc()
            )

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
