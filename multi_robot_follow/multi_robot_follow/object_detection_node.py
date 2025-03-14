import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO  # Install with: pip install ultralytics

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/drone/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, '/object_detections', 10)
        self.bridge = CvBridge()
        # Update the model path to your custom YOLOv8 model
        self.model = YOLO('/path/to/your/custom_model.pt')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        results = self.model(cv_image)
        detection_str = "Detected objects: "
        for result in results:
            for box in result.boxes:
                detection_str += f"[Class: {box.cls}, Conf: {box.confidence:.2f}] "

        self.publisher.publish(String(data=detection_str))
        self.get_logger().info(detection_str)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
