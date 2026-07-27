import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class DroneEnv(gym.Env, Node):
    """Custom Environment that follows gym interface and bridges with ROS 2"""
    def __init__(self):
        gym.Env.__init__(self)
        Node.__init__(self, 'drone_rl_env')
        
        # Action space: [linear_x, angular_z] (continuous)
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        
        # Observation space: 360 lidar rays + dist_to_goal + angle_to_goal = 362
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(362,), dtype=np.float32)
        
        # ROS 2 Pub/Sub
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.scan_data = np.zeros(360, dtype=np.float32)
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_yaw = 0.0
        
        self.target_x = 5.0
        self.target_y = 0.0
        
        self.collision_distance = 0.3
        self.goal_distance = 0.5
        
        self.prev_dist = None
        
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        # Clean invalid data
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0
        
        if len(ranges) >= 360:
            # Simple downsample if necessary or take first 360
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
        
    def step(self, action):
        # Apply the chosen velocity action
        vel_msg = Twist()
        vel_msg.linear.x = float(action[0])
        vel_msg.angular.z = float(action[1])
        self.vel_pub.publish(vel_msg)
        
        # Advance simulation (in actual setup, we might wait for gazebo tick)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # State Calculation
        dist_to_goal = math.hypot(self.target_x - self.drone_x, self.target_y - self.drone_y)
        angle_to_goal = math.atan2(self.target_y - self.drone_y, self.target_x - self.drone_x)
        relative_angle = angle_to_goal - self.drone_yaw
        relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
        
        state = np.concatenate((self.scan_data, [dist_to_goal, relative_angle])).astype(np.float32)
        
        # Reward Function
        reward = -0.05  # Time step penalty to encourage speed
        
        terminated = False
        truncated = False
        
        # Reward for moving closer to goal
        if self.prev_dist is not None:
            reward += (self.prev_dist - dist_to_goal) * 10.0
        self.prev_dist = dist_to_goal
        
        # Check Success
        if dist_to_goal < self.goal_distance:
            reward += 100.0
            terminated = True
            self.get_logger().info("Success! Goal Reached.")
            
        # Check Collision
        min_lidar = np.min(self.scan_data)
        if min_lidar < self.collision_distance:
            reward -= 100.0
            terminated = True
            self.get_logger().info("Crash! Collision detected.")
            
        return state, float(reward), terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Stop drone
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)
        
        # Here we would invoke a Gazebo service call to reset the drone position 
        # e.g., using /reset_simulation or setting entity state.
        # For this skeleton, we assume it's reset.
        
        self.target_x = np.random.uniform(2.0, 8.0)
        self.target_y = np.random.uniform(-4.0, 4.0)
        self.prev_dist = None
        
        # Spin to refresh topics
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
            
        dist_to_goal = math.hypot(self.target_x - self.drone_x, self.target_y - self.drone_y)
        angle_to_goal = math.atan2(self.target_y - self.drone_y, self.target_x - self.drone_x)
        relative_angle = angle_to_goal - self.drone_yaw
        relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
        
        state = np.concatenate((self.scan_data, [dist_to_goal, relative_angle])).astype(np.float32)
        return state, {}
