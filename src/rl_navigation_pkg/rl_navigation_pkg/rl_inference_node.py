import rclpy
from rclpy.node import Node
from stable_baselines3 import PPO
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math
import os

class RLInferenceNode(Node):
    def __init__(self):
        super().__init__('rl_inference_node')
        
        # Load the model
        model_path = os.path.join(os.getcwd(), "ppo_drone_nav.zip")
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at {model_path}!")
            raise FileNotFoundError("Model file not found.")
            
        self.model = PPO.load(model_path)
        self.get_logger().info(f"Loaded PPO model from {model_path}")
        
        # ROS 2 Pub/Sub
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.scan_data = np.zeros(360, dtype=np.float32)
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_yaw = 0.0
        
        # Dummy target goal for inference testing (this would normally come from a global planner or GCS)
        self.target_x = 5.0
        self.target_y = 5.0

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0
        if len(ranges) >= 360:
            self.scan_data = ranges[:360]
        else:
            self.scan_data = np.pad(ranges, (0, 360 - len(ranges)), 'constant', constant_values=10.0)

    def odom_callback(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def control_loop(self):
        # Construct state vector
        dist_to_goal = math.hypot(self.target_x - self.drone_x, self.target_y - self.drone_y)
        angle_to_goal = math.atan2(self.target_y - self.drone_y, self.target_x - self.drone_x)
        relative_angle = angle_to_goal - self.drone_yaw
        relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
        
        state = np.concatenate((self.scan_data, [dist_to_goal, relative_angle])).astype(np.float32)
        
        # Predict action
        action, _states = self.model.predict(state, deterministic=True)
        
        # Publish velocity
        vel_msg = Twist()
        vel_msg.linear.x = float(action[0])
        vel_msg.angular.z = float(action[1])
        self.vel_pub.publish(vel_msg)
        
        # Stop condition
        if dist_to_goal < 0.5:
            self.get_logger().info("Goal Reached. Stopping.")
            stop_msg = Twist()
            self.vel_pub.publish(stop_msg)
            # You might want to stop the timer or cancel navigation
            # self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = RLInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
