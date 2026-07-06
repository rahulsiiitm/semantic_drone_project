import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import numpy as np
import math
from enum import Enum
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FlightState(Enum):
    IDLE = 0
    TAKEOFF = 1
    NAVIGATE = 2
    LAND = 3

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
        self.emergency_stop_subscriber = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Nav2 Action Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


        # State Variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.offboard_setpoint_counter = 0
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.drone_yaw = 0.0
        self.flight_state = FlightState.IDLE
        self.emergency_stop_active = False
        
        # Target Variables
        self.target_altitude = -5.0
        self.nav2_velocity = np.array([0.0, 0.0])
        
        # Reactive Control Variables
        self.avoidance_vector = np.array([0.0, 0.0, 0.0])

        # Timers
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz control loop

    def vehicle_status_callback(self, vehicle_status):
        self.nav_state = vehicle_status.nav_state

    def estop_callback(self, msg):
        self.emergency_stop_active = msg.data

    def odometry_callback(self, msg):
        self.drone_position[0] = msg.position[0]
        self.drone_position[1] = msg.position[1]
        self.drone_position[2] = msg.position[2]

        # Extract yaw from quaternion
        q = msg.q
        self.drone_yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg):
        # Nav2 outputs /cmd_vel in the base_link frame (X is forward, Y is lateral)
        # We must convert this to the global NED frame for PX4
        rot_matrix = np.array([
            [math.cos(self.drone_yaw), -math.sin(self.drone_yaw)],
            [math.sin(self.drone_yaw),  math.cos(self.drone_yaw)]
        ])
        
        body_vel = np.array([msg.linear.x, msg.linear.y])
        global_vel = rot_matrix.dot(body_vel)
        self.nav2_velocity = global_vel

        # Pass through yaw rate if Nav2 commands turning
        self.nav2_yaw_rate = msg.angular.z

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
        
        # PX4 REQUIRES unused fields to be explicitly set to NaN! 
        # Otherwise it thinks we are commanding position [0,0,0] (the ground).
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]

        if self.flight_state == FlightState.IDLE:
            msg.velocity = [0.0, 0.0, 0.0]
            msg.yaw = self.drone_yaw
        
        elif self.flight_state == FlightState.TAKEOFF:
            # Ascend to target altitude
            altitude_error = self.target_altitude - self.drone_position[2]
            ascend_speed = max(-1.5, min(1.5, altitude_error)) # P-controller for altitude
            msg.velocity = [0.0, 0.0, float(ascend_speed)]
            msg.yaw = self.drone_yaw
            
            if abs(altitude_error) < 0.5:
                self.get_logger().info('Takeoff complete. Switching to NAVIGATE.')
                self.flight_state = FlightState.NAVIGATE
                self.send_nav2_goal(5.0, 0.0) # Fly 5 meters forward automatically

        elif self.flight_state == FlightState.NAVIGATE:
            # Altitude holding
            altitude_error = self.target_altitude - self.drone_position[2]
            z_vel = max(-1.0, min(1.0, altitude_error))
            
            # Use Nav2 Velocities for X and Y
            waypoint_velocity_xy = self.nav2_velocity

            # 2. Add Avoidance Vector (Body Frame to Global Frame mapping simplified)
            rot_matrix = np.array([
                [math.cos(self.drone_yaw), -math.sin(self.drone_yaw)],
                [math.sin(self.drone_yaw),  math.cos(self.drone_yaw)]
            ])
            avoidance_xy = np.array([self.avoidance_vector[0], self.avoidance_vector[1]])
            global_avoidance_xy = rot_matrix.dot(avoidance_xy)

            # 3. Final Velocity Command
            if self.emergency_stop_active:
                waypoint_velocity_xy = np.array([0.0, 0.0])
                global_avoidance_xy = np.array([0.0, 0.0])
                self.get_logger().warn('EMERGENCY STOP ACTIVE! Halting horizontal flight.', throttle_duration_sec=1.0)
            
            final_velocity_xy = waypoint_velocity_xy + global_avoidance_xy
            
            msg.velocity = [float(final_velocity_xy[0]), float(final_velocity_xy[1]), float(z_vel)]
            
            # Use Nav2's commanded yaw rate to turn the drone
            # Integrating yaw rate into a target yaw is complex in offboard velocity control, 
            # so we just let Nav2 command velocities while facing forward for now, 
            # or we slowly turn the yaw based on nav2_yaw_rate.
            if hasattr(self, 'nav2_yaw_rate'):
                msg.yaw = self.drone_yaw + (self.nav2_yaw_rate * 0.1) # integrate yaw rate
            else:
                msg.yaw = self.drone_yaw
                
        elif self.flight_state == FlightState.LAND:
            # Descend
            msg.velocity = [0.0, 0.0, 1.0] # 1.0 m/s down
            msg.yaw = self.drone_yaw
            
            # Simple landing detection: if we are close to ground (z ~ 0 or positive)
            if self.drone_position[2] > -0.2:
                self.get_logger().info('Touchdown detected. Disarming.')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0) # Disarm
                self.flight_state = FlightState.IDLE

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def send_nav2_goal(self, x, y):
        self.get_logger().info(f'Sending Nav2 Goal: x={x}, y={y}')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Orientation (facing forward)
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return

        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Nav2 Goal rejected.')
            return

        self.get_logger().info('Nav2 Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Nav2 Goal Reached! Switching to LAND.')
        self.flight_state = FlightState.LAND

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
        # Wait 2 seconds (20 ticks) to ensure PX4 has a steady stream of setpoints before arming
        if self.offboard_setpoint_counter == 20:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # Offboard mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0) # Arm
            self.flight_state = FlightState.TAKEOFF
            self.get_logger().info('Arming command sent. Offboard mode enabled. Starting TAKEOFF.')
        
        # 2. Publish offboard heartbeat (must be published before switching to offboard)
        self.publish_offboard_control_mode()
        
        # 3. Publish setpoints
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter < 21:
            self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
