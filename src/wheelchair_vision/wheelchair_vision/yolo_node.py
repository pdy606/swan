import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import math  # 객체 사이의 거리를 계산하기 위해 수학 모듈 추가!
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 1)
        self.image_pub = self.create_publisher(Image, '/yolo/image_raw', 10)
        self.label_pub = self.create_publisher(String, '/yolo/detected_objects', 10)
        
        self.bridge = CvBridge()
        
        package_share = get_package_share_directory('wheelchair_vision')
        model_path = os.path.join(package_share, 'wheelchair_vision', 'best.pt')
        
        if os.path.exists(model_path):
            self.get_logger().info(f'Loading custom model: {model_path}')
            self.model = YOLO(model_path)
        else:
            self.get_logger().warn('best.pt not found! Using default yolov8n.pt')
            self.model = YOLO('yolov8n.pt')

        self.bev_matrix = None
        self.max_dist_m = 6.0 
        self.horizon_ratio = 0.54

    def init_bev_matrix(self, h, w):
        src_pts = np.float32([
            [w * 0.15, h], [w * 0.85, h],
            [w * 0.60, h * self.horizon_ratio], [w * 0.40, h * self.horizon_ratio]
        ])
        dst_pts = np.float32([
            [w * 0.25, h], [w * 0.75, h],
            [w * 0.75, 0], [w * 0.25, 0]
        ])
        self.bev_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            h, w = cv_image.shape[:2]
            if self.bev_matrix is None:
                self.init_bev_matrix(h, w)
            
            results = self.model(cv_image, conf=0.20)
            annotated_frame = results[0].plot()
            
            detected_objects = [] # 감지된 객체의 3D 좌표를 저장할 리스트
            
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    x_center = (x1 + x2) / 2.0
                    y_center = (y1 + y2) / 2.0
                    y_bottom = y2
                    box_h = y2 - y1
                    
                    horizon_y = h * self.horizon_ratio
                    
                    # --- 1. 직진 거리(Z)와 측면 거리(X) 3D 좌표 추출 ---
                    if y_bottom > horizon_y:
                        # 바닥 물체 (BEV)
                        pt = np.array([[[x_center, y_bottom]]], dtype=np.float32)
                        bev_pt = cv2.perspectiveTransform(pt, self.bev_matrix)
                        bev_x, bev_y = bev_pt[0][0]
                        bev_y = np.clip(bev_y, 0, h)
                        
                        dist_z = (h - bev_y) / h * self.max_dist_m
                        # 화면 중앙(w/2)을 0m로 두고, 좌우 측면 거리 계산 (비례식)
                        dist_x = ((bev_x - w / 2.0) / w) * 8.0 
                    else:
                        # 공중 물체 (신호등 등)
                        dy = horizon_y - y_bottom
                        if dy > 0:
                            composite_feature = box_h + (dy * 0.5)
                            dist_z = 250.0 / max(composite_feature, 1.0)
                            dist_x = ((x_center - w / 2.0) / w) * (dist_z * 1.5)
                        else:
                            dist_z = self.max_dist_m
                            dist_x = 0.0
                    
                    dist_z = max(0.0, dist_z)
                    
                    # 6m 이내에 있는 유의미한 물체만 분석 대상에 포함
                    if dist_z < 6.0:
                        detected_objects.append({
                            'class': class_name,
                            'x': dist_x,
                            'z': dist_z,
                            'box': (int(x1), int(y1), int(x2), int(y2)),
                            'center': (int(x_center), int(y_center))
                        })

            # --- 2. 가장 가까운 순서(Z거리)대로 정렬 후 번호 부여 ---
            detected_objects.sort(key=lambda obj: obj['z'])
            detected_class_names = []

            for i, obj in enumerate(detected_objects):
                rank = i + 1 # 1번, 2번, 3번...
                x1, y1, x2, y2 = obj['box']
                detected_class_names.append(f"#{rank} {obj['class']}")
                
                label = f"#{rank} {obj['class']}: {obj['z']:.1f}m"
                text_y = y1 + 25 if y1 < 20 else y1 - 10
                
                # 순위표 강조 텍스트 
                cv2.putText(annotated_frame, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
                cv2.putText(annotated_frame, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # --- 3. 객체 사이의 틈새 거리(Gap) 측정 후 선 긋기 ---
            for i in range(len(detected_objects) - 1):
                obj_A = detected_objects[i]
                obj_B = detected_objects[i+1]
                
                # 피타고라스 정리로 두 물체 사이의 실제 물리적 대각선 거리 계산
                gap_m = math.sqrt((obj_A['x'] - obj_B['x'])**2 + (obj_A['z'] - obj_B['z'])**2)
                
                pt_A = obj_A['center']
                pt_B = obj_B['center']
                
                # 두 물체 중심을 잇는 형광 핑크색 선 긋기
                cv2.line(annotated_frame, pt_A, pt_B, (255, 0, 255), 2)
                
                # 선 한가운데에 거리(Gap) 텍스트 띄우기
                mid_x = (pt_A[0] + pt_B[0]) // 2
                mid_y = (pt_A[1] + pt_B[1]) // 2
                gap_label = f"Gap: {gap_m:.1f}m"
                
                cv2.putText(annotated_frame, gap_label, (mid_x, mid_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(annotated_frame, gap_label, (mid_x, mid_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 105, 180), 2)

            if detected_class_names:
                msg_str = String()
                msg_str.data = f"Detected Order: {', '.join(detected_class_names)}"
                self.label_pub.publish(msg_str)
            
            annotated_msg = Image()
            annotated_msg.header = msg.header
            annotated_msg.height = annotated_frame.shape[0]
            annotated_msg.width = annotated_frame.shape[1]
            annotated_msg.encoding = 'bgr8'
            annotated_msg.is_bigendian = 0
            annotated_msg.step = annotated_frame.shape[1] * 3
            annotated_msg.data = np.ascontiguousarray(annotated_frame).tobytes()
            
            self.image_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()