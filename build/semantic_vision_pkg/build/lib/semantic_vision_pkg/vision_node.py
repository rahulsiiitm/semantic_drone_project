import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(Image, 'semantic_mask', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n-seg.pt')
        self.get_logger().info('Vision Node Started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Resize for consistency
            frame = cv2.resize(frame, (640, 480))
            
            # Run Inference
            results = self.model(frame, verbose=False)
            
            # Simple Logic: If mask exists, publish it
            # (In real implementation, we would process masks to IDs)
            if results[0].masks:
                # For MVP, just sending the raw annotated image as a placeholder
                # Real version sends integer mask
                annotated_frame = results[0].plot()
                msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
                self.publisher_.publish(msg)

def run_vision_node(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
