import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # 1. 카메라 영상 구독 (Topic)
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        # 2. 결과 발행 (Topic)
        self.image_pub = self.create_publisher(Image, '/yolo/image_raw', 10)
        self.label_pub = self.create_publisher(String, '/yolo/detected_objects', 10)
        
        self.bridge = CvBridge()
        
        # 3. 모델 로드 (best.pt가 없을 땐 기본 yolov8n.pt로 자동 테스트)
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
            results = self.model(cv_image, conf=0.5)
            
            detected_classes = []
            for r in results:
                cv_image = r.plot()
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    detected_classes.append(class_name)
            
            if detected_classes:
                msg_str = String()
                msg_str.data = f"Detected: {', '.join(set(detected_classes))}"
                self.label_pub.publish(msg_str)
            
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
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