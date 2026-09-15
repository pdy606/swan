import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
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
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR) # 가제보 색상 버그 픽스
            
            h, w = cv_image.shape[:2]
            if self.bev_matrix is None:
                self.init_bev_matrix(h, w)
            
            results = self.model(cv_image, conf=0.30)
            detected_classes = []
            annotated_frame = results[0].plot()
            
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    detected_classes.append(class_name)

                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    x_center = (x1 + x2) / 2.0
                    y_bottom = y2
                    box_h = y2 - y1  # 💡 물체의 화면상 크기(높이)
                    
                    horizon_y = h * self.horizon_ratio
                    
                    if y_bottom > horizon_y:
                        # 1. 바닥에 있는 물체 -> 조감도(BEV) 평면 투시 정밀 변환
                        pt = np.array([[[x_center, y_bottom]]], dtype=np.float32)
                        bev_pt = cv2.perspectiveTransform(pt, self.bev_matrix)
                        bev_y = np.clip(bev_pt[0][0][1], 0, h)
                        distance_m = (h - bev_y) / h * self.max_dist_m
                    else:
                        # 2. 공중에 떠 있는 물체(신호등) -> 크기(box_h)와 위치(dy) 종합 융합 계산!
                        dy = horizon_y - y_bottom
                        
                        if dy > 0:
                            # 💡 종합 특성값(Composite Feature): 박스가 커지는 것과 위로 올라가는 것을 동시에 반영
                            # 크기 변화가 더 직관적이므로 box_h에 가중치를 더 줌
                            composite_feature = box_h + (dy * 0.5)
                            
                            # 250.0은 가제보 카메라 화각(FOV)에 맞춘 튜닝 상수
                            distance_m = 250.0 / max(composite_feature, 1.0)
                        else:
                            distance_m = self.max_dist_m
                    
                    distance_m = max(0.0, distance_m)
                    
                    if distance_m >= 6.0:
                        label = "Dist: Far"
                    else:
                        label = f"Dist: {distance_m:.1f}m"
                    
                    text_y = int(y1) + 25 if y1 < 20 else int(y1) - 10
                    cv2.putText(annotated_frame, label, (int(x1), text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
                    cv2.putText(annotated_frame, label, (int(x1), text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            if detected_classes:
                msg_str = String()
                msg_str.data = f"Detected: {', '.join(set(detected_classes))}"
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