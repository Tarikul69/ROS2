import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO  # YOLOv8 model

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.image_subscriber_ = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Load YOLO model
        self.get_logger().info("Object Detection Node Started")

    def image_callback(self, msg):
        # Convert ROS 2 image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run object detection
        results = self.model(frame)

        # Extract detected objects
        detected_objects = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                detected_objects.append(f"Class: {self.model.names[class_id]}, Confidence: {confidence:.2f}")

        # Publish detected objects
        detected_msg = String()
        detected_msg.data = ', '.join(detected_objects) if detected_objects else "No objects detected"
        self.publisher_.publish(detected_msg)

        # Show image with detections (Optional)
        result_frame = results[0].plot()
        cv2.imshow("Object Detection", result_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
