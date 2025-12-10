import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        # Publish to the topic the C++ plugin is listening to
        self.publisher_ = self.create_publisher(Image, '/semantic_mask', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.get_logger().info('Vision Node Active: SIMULATION MODE (No Webcam)')
        self.frame_count = 0

    def timer_callback(self):
        # 1. Create a blank image MATCHING COSTMAP SIZE (200x200)
        # 255 (White) = Safe Space
        frame = np.full((200, 200), 255, dtype=np.uint8)
        
        # 2. Toggle "Person" every 30 frames (approx 3 seconds)
        self.frame_count += 1
        if (self.frame_count // 30) % 2 == 1:
            # Draw a Black Box (Person) in the center
            # 0 (Black) = Lethal Obstacle
            # Coordinates are in the 200x200 map space
            cv2.rectangle(frame, (80, 80), (120, 120), 0, -1)
            self.get_logger().info('SIMULATING: Person Detected (Sending lethal mask)', throttle_duration_sec=2)
        else:
            self.get_logger().info('SIMULATING: Path Clear', throttle_duration_sec=2)

        # 3. Publish as MONO8 (Grayscale)
        # Critical: Costmaps expect 1-channel images, not RGB.
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="mono8")
        self.publisher_.publish(msg)

def run_vision_node(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_vision_node()