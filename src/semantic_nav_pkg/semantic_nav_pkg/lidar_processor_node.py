import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class LidarProcessorNode(Node):
    def __init__(self):
        super().__init__('lidar_processor_node')
        
        # Parameters
        self.declare_parameter('stop_distance', 1.5)
        self.stop_distance = self.get_parameter('stop_distance').value
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        self.get_logger().info(f"LiDAR Processor initialized. Stop distance: {self.stop_distance}m")

    def scan_callback(self, msg):
        emergency = False
        
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return
            
        # We check the front 120 degrees (-60 to +60 degrees roughly) to avoid stopping for things on the side or behind.
        # This translates to roughly -1.04 to +1.04 radians.
        
        for i, range_val in enumerate(msg.ranges):
            # Ignore infinite or invalid readings
            if math.isinf(range_val) or math.isnan(range_val):
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            
            # Check if the angle is within front -60 to +60 degrees
            if -1.04 < angle < 1.04:
                # Ensure the reading is within sensor limits and closer than stop_distance
                if msg.range_min < range_val < self.stop_distance:
                    emergency = True
                    break
        
        # Publish the emergency stop flag
        msg_out = Bool()
        msg_out.data = emergency
        self.estop_pub.publish(msg_out)
        
        if emergency:
            self.get_logger().warn('OBSTACLE IN STOP ZONE! Emergency stop active.', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
