import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class SemanticVisionNode(Node):
    def __init__(self):
        super().__init__('semantic_vision_node')
        
        # Declare parameters
        self.declare_parameter('use_webcam', True)
        self.use_webcam = self.get_parameter('use_webcam').value
        
        self.bridge = CvBridge()
        
        self.get_logger().info("Loading YOLOv8 model...")
        self.model = YOLO('yolov8n-seg.pt')
        self.get_logger().info("YOLOv8 loaded successfully.")
        
        # Publishers
        self.mask_pub = self.create_publisher(Image, '/semantic_mask', 10)
        self.debug_pub = self.create_publisher(Image, '/vision_debug', 10)
        
        if self.use_webcam:
            self.get_logger().info("Using Webcam (Device 0) for testing.")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error("Error: Webcam not found.")
            else:
                # Timer to read frames at ~30 FPS
                self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        else:
            self.get_logger().info("Subscribing to /camera/image_raw.")
            self.sub = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.process_frame(frame)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def process_frame(self, frame):
        # Resize for consistent processing
        frame = cv2.resize(frame, (640, 480))
        display_frame = frame.copy()
        
        # Blank mask for the costmap (0 = Free, 255 = Lethal Obstacle)
        mask = np.zeros((480, 640), dtype=np.uint8)
        
        # Run AI Inference
        results = self.model.predict(source=frame, save=False, conf=0.5, verbose=False)
        result = results[0]
        
        if result.boxes:
            classes = result.boxes.cls.cpu().numpy()
            boxes = result.boxes.xyxy.cpu().numpy()

            for i, cls_id in enumerate(classes):
                x1, y1, x2, y2 = map(int, boxes[i])
                cx, cy = int((x1+x2)/2), int((y1+y2)/2)

                # RULE: PERSON (ID 0) = LETHAL OBSTACLE (Proxy for Cable/Human)
                if cls_id == 0:
                    # 1. Update Semantic Mask (White circle on black background)
                    # We inflate the obstacle by 60 pixels to give the drone a safety margin
                    cv2.circle(mask, (cx, cy), 60, 255, -1)
                    
                    # 2. Draw Debug Graphics
                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(display_frame, "OBSTACLE DETECTED", (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # Draw a transparent safety bubble
                    overlay = display_frame.copy()
                    cv2.circle(overlay, (cx, cy), 60, (0, 255, 255), -1)
                    cv2.addWeighted(overlay, 0.3, display_frame, 0.7, 0, display_frame)

        # Publish the binary mask for Nav2
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        self.mask_pub.publish(mask_msg)
        
        # Publish the debug frame for RQT visualization
        debug_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
        self.debug_pub.publish(debug_msg)

    def destroy_node(self):
        if self.use_webcam and hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def run_vision_node(args=None):
    rclpy.init(args=args)
    node = SemanticVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_vision_node()