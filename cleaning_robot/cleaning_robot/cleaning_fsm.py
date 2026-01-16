#!/usr/bin/env python3
"""
CLEANING ROBOT FSM - ROS 2 Humble Version
==========================================
Robot pembersih otomatis seperti Roomba
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from enum import Enum
import math
import random


class State(Enum):
    IDLE = 0
    CLEANING = 1
    SPOT_CLEANING = 2
    EDGE_CLEANING = 3
    STUCK = 4
    DOCKING = 5
    CHARGING = 6


class CleaningRobot(Node):
    def __init__(self):
        super().__init__('cleaning_robot')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_level', 10)
        self.cleaning_pub = self.create_publisher(Bool, '/is_cleaning', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Timer 10 Hz
        self.timer = self.create_timer(0.1, self.fsm_loop)
        
        # State variables
        self.current_state = State.IDLE
        self.state_start_time = self.get_clock().now()
        
        # Robot parameters
        self.battery_level = 100.0
        self.battery_drain_rate = 0.15
        self.battery_charge_rate = 1.0
        self.low_battery_threshold = 20.0
        
        # Cleaning parameters
        self.area_cleaned = 0.0
        self.cleaning_efficiency = 0.3
        self.spot_clean_duration = 8.0
        self.edge_clean_duration = 12.0
        
        # Position tracking
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.dock_position = (0.0, 0.0)
        
        # Sensor data
        self.min_distance = float('inf')
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.wall_detected = False
        self.obstacle_detected = False
        
        # Movement tracking for stuck detection
        self.last_position = (0.0, 0.0)
        self.movement_check_time = self.get_clock().now()
        self.is_stuck = False
        
        # Speeds
        self.cleaning_speed = 0.2
        self.edge_speed = 0.15
        self.angular_speed = 0.5
        
        # Spiral cleaning
        self.spiral_radius = 0.0
        self.spiral_angle = 0.0
        
        self.get_logger().info('=== Cleaning Robot Initialized ===')
        self.get_logger().info(f'Battery: {self.battery_level}%')
        self.get_logger().info('Ready to clean!')
    
    def laser_callback(self, msg):
        """Process laser scan data"""
        if len(msg.ranges) == 0:
            return
        
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            
            # Front sensor (±15 degrees)
            front_start = len(msg.ranges) - 15
            front_end = 15
            front_ranges = list(msg.ranges[front_start:]) + list(msg.ranges[:front_end])
            front_valid = [r for r in front_ranges if msg.range_min < r < msg.range_max]
            self.front_distance = min(front_valid) if front_valid else float('inf')
            
            # Left sensor (75-105 degrees)
            left_ranges = msg.ranges[75:105]
            left_valid = [r for r in left_ranges if msg.range_min < r < msg.range_max]
            self.left_distance = min(left_valid) if left_valid else float('inf')
            
            # Right sensor (255-285 degrees)
            right_ranges = msg.ranges[255:285]
            right_valid = [r for r in right_ranges if msg.range_min < r < msg.range_max]
            self.right_distance = min(right_valid) if right_valid else float('inf')
            
            # Wall detection
            self.wall_detected = (self.left_distance < 0.35 or self.right_distance < 0.35)
            
            # Obstacle detection
            self.obstacle_detected = self.front_distance < 0.3
    
    def odom_callback(self, msg):
        """Track robot position"""
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Check if stuck
        self.check_stuck()
    
    def check_stuck(self):
        """Detect if robot is stuck"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.movement_check_time).nanoseconds / 1e9
        
        if time_diff > 3.0:
            dx = self.position[0] - self.last_position[0]
            dy = self.position[1] - self.last_position[1]
            movement = math.sqrt(dx*dx + dy*dy)
            
            if movement < 0.05 and self.current_state == State.CLEANING:
                self.is_stuck = True
            else:
                self.is_stuck = False
            
            self.last_position = self.position
            self.movement_check_time = current_time
    
    def change_state(self, new_state):
        if new_state != self.current_state:
            self.get_logger().info(f'State: {self.current_state.name} -> {new_state.name}')
            self.current_state = new_state
            self.state_start_time = self.get_clock().now()
            self.spiral_radius = 0.0
    
    def get_elapsed_time(self):
        current_time = self.get_clock().now()
        return (current_time - self.state_start_time).nanoseconds / 1e9
    
    def publish_velocity(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
    
    def update_battery(self, rate_multiplier=1.0):
        self.battery_level -= self.battery_drain_rate * rate_multiplier * 0.1
        self.battery_level = max(0.0, min(100.0, self.battery_level))
        
        msg = Float32()
        msg.data = self.battery_level
        self.battery_pub.publish(msg)
    
    def update_cleaning_area(self):
        self.area_cleaned += self.cleaning_efficiency * 0.1
    
    def distance_to_dock(self):
        dx = self.dock_position[0] - self.position[0]
        dy = self.dock_position[1] - self.position[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def state_idle(self):
        """Waiting to start cleaning"""
        self.publish_velocity(0.0, 0.0)
        
        msg = Bool()
        msg.data = False
        self.cleaning_pub.publish(msg)
        
        self.get_logger().info(f'IDLE - Battery: {self.battery_level:.1f}%', throttle_duration_sec=5.0)
        
        if self.get_elapsed_time() > 3.0 and self.battery_level > self.low_battery_threshold:
            self.get_logger().info('Starting cleaning cycle!')
            self.change_state(State.CLEANING)
    
    def state_cleaning(self):
        """Main cleaning state with random pattern"""
        msg = Bool()
        msg.data = True
        self.cleaning_pub.publish(msg)
        
        # Check battery
        if self.battery_level < self.low_battery_threshold:
            self.get_logger().warn(f'Low battery: {self.battery_level:.1f}%')
            self.change_state(State.DOCKING)
            return
        
        # Check stuck
        if self.is_stuck:
            self.get_logger().warn('Robot stuck!')
            self.change_state(State.STUCK)
            return
        
        # Random transition to edge cleaning
        if self.wall_detected and random.random() < 0.05:
            self.get_logger().info('Wall detected - Edge cleaning mode')
            self.change_state(State.EDGE_CLEANING)
            return
        
        # Random spot cleaning
        if random.random() < 0.02:
            self.get_logger().info('Dirty spot detected!')
            self.change_state(State.SPOT_CLEANING)
            return
        
        # Random movement pattern
        if self.obstacle_detected:
            turn_direction = random.choice([-1, 1])
            self.publish_velocity(0.0, self.angular_speed * turn_direction)
            self.get_logger().info('Avoiding obstacle...', throttle_duration_sec=2.0)
        else:
            random_turn = random.uniform(-0.15, 0.15)
            self.publish_velocity(self.cleaning_speed, random_turn)
        
        self.update_battery(1.0)
        self.update_cleaning_area()
        
        elapsed = int(self.get_elapsed_time())
        if elapsed > 0 and elapsed % 10 == 0:
            self.get_logger().info(
                f'Cleaning... Area: {self.area_cleaned:.1f}m² | Battery: {self.battery_level:.1f}%'
            )
    
    def state_spot_cleaning(self):
        """Intensive cleaning with spiral pattern"""
        msg = Bool()
        msg.data = True
        self.cleaning_pub.publish(msg)
        
        self.get_logger().info('Spot cleaning...', throttle_duration_sec=2.0)
        
        # Spiral pattern
        if self.spiral_radius < 1.5:
            self.spiral_radius += 0.015
            angular_vel = 1.0 / (self.spiral_radius + 0.1)
            self.publish_velocity(self.cleaning_speed * 0.4, angular_vel)
        else:
            self.get_logger().info('Spot cleaning complete')
            self.change_state(State.CLEANING)
        
        self.update_battery(1.5)
        self.update_cleaning_area()
        
        if self.get_elapsed_time() > self.spot_clean_duration:
            self.change_state(State.CLEANING)
    
    def state_edge_cleaning(self):
        """Clean along walls"""
        msg = Bool()
        msg.data = True
        self.cleaning_pub.publish(msg)
        
        self.get_logger().info('Edge cleaning...', throttle_duration_sec=2.0)
        
        if self.wall_detected:
            # Follow wall
            if self.left_distance < self.right_distance:
                # Wall on left
                self.publish_velocity(self.edge_speed, -0.1)
            else:
                # Wall on right
                self.publish_velocity(self.edge_speed, 0.1)
        else:
            # Lost wall
            self.get_logger().info('Edge cleaning complete')
            self.change_state(State.CLEANING)
            return
        
        self.update_battery(1.2)
        self.update_cleaning_area()
        
        if self.get_elapsed_time() > self.edge_clean_duration:
            self.change_state(State.CLEANING)
    
    def state_stuck(self):
        """Recovery from stuck position"""
        msg = Bool()
        msg.data = False
        self.cleaning_pub.publish(msg)
        
        self.get_logger().warn('STUCK - Recovering...', throttle_duration_sec=1.0)
        
        if self.get_elapsed_time() < 1.5:
            # Reverse
            self.publish_velocity(-0.15, 0.0)
        elif self.get_elapsed_time() < 3.0:
            # Turn
            self.publish_velocity(0.0, 0.8)
        else:
            self.get_logger().info('Recovery complete')
            self.is_stuck = False
            self.change_state(State.CLEANING)
        
        self.update_battery(0.5)
    
    def state_docking(self):
        """Return to charging dock"""
        msg = Bool()
        msg.data = False
        self.cleaning_pub.publish(msg)
        
        self.get_logger().info(f'Searching for dock... Battery: {self.battery_level:.1f}%', 
                              throttle_duration_sec=2.0)
        
        distance = self.distance_to_dock()
        
        if distance < 0.15:
            self.get_logger().info('Docked successfully!')
            self.change_state(State.CHARGING)
        else:
            # Simple navigation to dock
            if self.obstacle_detected:
                self.publish_velocity(0.0, self.angular_speed)
            else:
                # Calculate angle to dock
                dx = self.dock_position[0] - self.position[0]
                dy = self.dock_position[1] - self.position[1]
                target_angle = math.atan2(dy, dx)
                angle_diff = target_angle - self.yaw
                
                # Normalize
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi
                
                if abs(angle_diff) > 0.2:
                    angular_vel = 0.4 if angle_diff > 0 else -0.4
                    self.publish_velocity(0.0, angular_vel)
                else:
                    self.publish_velocity(0.15, angle_diff * 0.5)
        
        self.update_battery(0.3)
    
    def state_charging(self):
        """Charging at dock"""
        self.publish_velocity(0.0, 0.0)
        
        msg = Bool()
        msg.data = False
        self.cleaning_pub.publish(msg)
        
        # Charge
        self.battery_level += self.battery_charge_rate * 0.1
        self.battery_level = min(100.0, self.battery_level)
        
        self.get_logger().info(f'Charging... {self.battery_level:.1f}%', throttle_duration_sec=3.0)
        
        if self.battery_level >= 95.0:
            self.get_logger().info('Fully charged!')
            self.get_logger().info(f'Total area cleaned: {self.area_cleaned:.1f}m²')
            self.change_state(State.IDLE)
    
    def fsm_loop(self):
        """Main FSM loop"""
        if self.current_state == State.IDLE:
            self.state_idle()
        elif self.current_state == State.CLEANING:
            self.state_cleaning()
        elif self.current_state == State.SPOT_CLEANING:
            self.state_spot_cleaning()
        elif self.current_state == State.EDGE_CLEANING:
            self.state_edge_cleaning()
        elif self.current_state == State.STUCK:
            self.state_stuck()
        elif self.current_state == State.DOCKING:
            self.state_docking()
        elif self.current_state == State.CHARGING:
            self.state_charging()


def main(args=None):
    rclpy.init(args=args)
    cleaning_robot = CleaningRobot()
    
    try:
        rclpy.spin(cleaning_robot)
    except KeyboardInterrupt:
        pass
    finally:
        cleaning_robot.publish_velocity(0.0, 0.0)
        cleaning_robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
