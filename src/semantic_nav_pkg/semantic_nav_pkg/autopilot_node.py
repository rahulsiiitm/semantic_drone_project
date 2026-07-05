import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from sensor_msgs.msg import Image
import numpy as np
import math

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')

        # Configure QoS profile for PX4 topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
        self.semantic_mask_subscriber = self.create_subscription(
            Image, '/semantic_mask', self.mask_callback, 10)

        # State Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.offboard_setpoint_counter = 0
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.drone_yaw = 0.0

        # Destination Waypoint (NED coordinates: North, East, Down)
        # Assuming starting at 0, 0, we want to fly North by 20 meters, at 5m height.
        self.target_waypoint = np.array([20.0, 0.0, -5.0])
        
        # Reactive Control Variables
        self.avoidance_vector = np.array([0.0, 0.0, 0.0])

        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz control loop

    def vehicle_status_callback(self, vehicle_status):
        self.nav_state = vehicle_status.nav_state

    def odometry_callback(self, msg):
        self.drone_position[0] = msg.position[0]
        self.drone_position[1] = msg.position[1]
        self.drone_position[2] = msg.position[2]

        # Extract yaw from quaternion
        q = msg.q
        self.drone_yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))

    def mask_callback(self, msg):
        # Convert ROS Image to Numpy array
        mask = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
        
        # Analyze the mask for obstacles (White pixels = 255)
        # Divide image into 3 vertical zones: Left, Center, Right
        h, w = mask.shape
        third = w // 3
        
        left_zone = mask[:, :third]
        center_zone = mask[:, third:2*third]
        right_zone = mask[:, 2*third:]

        left_score = np.sum(left_zone == 255)
        center_score = np.sum(center_zone == 255)
        right_score = np.sum(right_zone == 255)

        # Reactive Avoidance Logic
        self.avoidance_vector = np.array([0.0, 0.0, 0.0])
        threshold = 1000 # Minimum pixels to consider an obstacle

        if center_score > threshold:
            self.get_logger().info('OBSTACLE DEAD AHEAD! Dodging...')
            if left_score < right_score:
                # Steer Left
                self.avoidance_vector = np.array([0.0, -2.0, 0.0]) # 2 m/s Left
            else:
                # Steer Right
                self.avoidance_vector = np.array([0.0, 2.0, 0.0]) # 2 m/s Right
        else:
            # Clear path
            self.avoidance_vector = np.array([0.0, 0.0, 0.0])

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        # 1. Calculate Vector to Waypoint (Global Frame)
        direction = self.target_waypoint - self.drone_position
        distance = np.linalg.norm(direction)
        
        if distance > 0.5:
            direction_normalized = direction / distance
            # Cap speed at 2 m/s
            waypoint_velocity = direction_normalized * min(distance, 2.0) 
        else:
            waypoint_velocity = np.array([0.0, 0.0, 0.0]) # Reached destination

        # 2. Add Avoidance Vector (Body Frame to Global Frame mapping simplified)
        # Avoidance vector is relative to drone (Y is right/left). 
        # Rotate avoidance vector by drone's yaw to get global vector.
        rot_matrix = np.array([
            [math.cos(self.drone_yaw), -math.sin(self.drone_yaw), 0],
            [math.sin(self.drone_yaw),  math.cos(self.drone_yaw), 0],
            [0, 0, 1]
        ])
        global_avoidance = rot_matrix.dot(self.avoidance_vector)

        # 3. Final Velocity Command
        final_velocity = waypoint_velocity + global_avoidance
        
        msg.velocity = [float(final_velocity[0]), float(final_velocity[1]), float(final_velocity[2])]
        msg.yaw = math.atan2(final_velocity[1], final_velocity[0]) # Point nose towards movement direction
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kwargs.get("param1", 0.0)
        msg.param2 = kwargs.get("param2", 0.0)
        msg.param3 = kwargs.get("param3", 0.0)
        msg.param4 = kwargs.get("param4", 0.0)
        msg.param5 = kwargs.get("param5", 0.0)
        msg.param6 = kwargs.get("param6", 0.0)
        msg.param7 = kwargs.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        # 1. Arm and switch to offboard mode
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # Offboard mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # Arm
        
        # 2. Publish offboard heartbeat (must be published before switching to offboard)
        self.publish_offboard_control_mode()
        
        # 3. Publish setpoints
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
