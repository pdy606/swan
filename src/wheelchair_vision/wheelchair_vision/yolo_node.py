import rclpy
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
        
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        self.image_pub = self.create_publisher(Image, '/yolo/image_raw', 10)
        self.label_pub = self.create_publisher(String, '/yolo/detected_objects', 10)
        
        self.bridge = CvBridge()
        
        # 3. 다영이가 직접 학습시킨 커스텀 모델(best.pt) 로드!
        model_path = '/home/userpdy606/swan/src/wheelchair_vision/wheelchair_vision/best.pt'
        if os.path.exists(model_path):
            self.get_logger().info(f'Loading custom model: {model_path}')
            self.model = YOLO(model_path)
        else:
            self.get_logger().warn('best.pt not found! Using default yolov8n.pt for testing.')
            self.model = YOLO('yolov8n.pt')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 가제보 화면(도메인 갭)을 이겨내기 위해 conf=0.1로 낮게 유지
            results = self.model(cv_image, conf=0.35)
            
            detected_classes = []
            annotated_frame = results[0].plot()
            
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    detected_classes.append(class_name)
            
            if detected_classes:
                msg_str = String()
                msg_str.data = f"Detected: {', '.join(set(detected_classes))}"
                self.label_pub.publish(msg_str)
            
            # 🌟 통역사(cv_bridge) 대신 직접 수동 포장! (에러 16 철벽 방어)
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