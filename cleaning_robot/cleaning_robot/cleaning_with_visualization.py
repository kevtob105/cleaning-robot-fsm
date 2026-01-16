#!/usr/bin/env python3
"""
CLEANING ROBOT dengan VISUALISASI DEBU
=======================================
Robot cleaning + marker untuk menunjukkan area yang sudah dibersihkan
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray
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


class CleaningRobotWithViz(Node):
    def __init__(self):
        super().__init__('cleaning_robot_viz')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_level', 10)
        self.cleaning_pub = self.create_publisher(Bool, '/is_cleaning', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/cleaning_visualization', 10)
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.viz_timer = self.create_timer(0.5, self.update_visualization)
        
        # State variables
        self.current_state = State.IDLE
        self.state_start_time = self.get_clock().now()
        
        # Robot parameters
        self.battery_level = 100.0
        self.battery_charge_rate = 2.0  
        self.battery_drain_rate = 0.15
        self.low_battery_threshold = 30.0
        
        # Position tracking
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.dock_position = (0.0, 0.0)
        
        # Sensor data
        self.min_distance = float('inf')
        self.front_distance = float('inf')
        self.wall_detected = False
        self.obstacle_detected = False
        self.emergency_stop = False         
        # Movement tracking
        self.last_position = (0.0, 0.0)
        self.movement_check_time = self.get_clock().now()
        self.is_stuck = False
        
        # Speeds
        self.cleaning_speed = 0.2
        self.angular_speed = 0.5
        
        # Spiral cleaning
        self.spiral_radius = 0.0
        
        # VISUALIZATION DATA
        self.dirty_spots = []  # List of (x, y) positions with "dirt"
        self.cleaned_areas = []  # List of (x, y) positions that are cleaned
        self.cleaning_path = []  # Path followed by robot
        self.area_cleaned = 0.0
        
        # Generate dirty spots (debu)
        self.generate_dirty_spots()
        
        self.get_logger().info('=== Cleaning Robot with Visualization ===')
        self.get_logger().info(f'Generated {len(self.dirty_spots)} dirty spots')
        self.get_logger().info('Open RViz to see cleaning visualization!')
    
    def generate_dirty_spots(self):
        """Generate random dirty spots (debu) in the environment"""
        # Create grid of dirty spots
        for x in range(-4, 5):
            for y in range(-4, 5):
                # Skip area near dock
                if abs(x) < 1 and abs(y) < 1:
                    continue
                
                # Random spots (70% chance)
                if random.random() < 0.7:
                    # Add some randomness to position
                    pos_x = x + random.uniform(-0.3, 0.3)
                    pos_y = y + random.uniform(-0.3, 0.3)
                    self.dirty_spots.append((pos_x, pos_y))
        
        self.get_logger().info(f'Created {len(self.dirty_spots)} dirty spots!')
    
    def check_and_clean_spots(self):
        """Check if robot is near dirty spots and clean them"""
        cleaning_radius = 0.4  # Robot can clean within 0.4m radius
        
        cleaned_this_cycle = []
        
        for spot in self.dirty_spots:
            dx = spot[0] - self.position[0]
            dy = spot[1] - self.position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < cleaning_radius:
                cleaned_this_cycle.append(spot)
                self.cleaned_areas.append(spot)
                self.area_cleaned += 0.25  # 0.25 m² per spot
        
        # Remove cleaned spots
        for spot in cleaned_this_cycle:
            self.dirty_spots.remove(spot)
        
        if len(cleaned_this_cycle) > 0:
            self.get_logger().info(f'✨ Cleaned {len(cleaned_this_cycle)} spots! Remaining: {len(self.dirty_spots)}')
    
    def update_visualization(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        # 1. Dirty spots (RED spheres)
        for i, spot in enumerate(self.dirty_spots):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dirty_spots"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = spot[0]
            marker.pose.position.y = spot[1]
            marker.pose.position.z = 0.05
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.05
            marker.color.r = 0.6
            marker.color.g = 0.3
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        
        # 2. Cleaned areas (GREEN spheres)
        for i, spot in enumerate(self.cleaned_areas):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cleaned_spots"
            marker.id = i + 10000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = spot[0]
            marker.pose.position.y = spot[1]
            marker.pose.position.z = 0.02
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.02
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3
            marker_array.markers.append(marker)
        
        # 3. Cleaning path (BLUE line)
        if len(self.cleaning_path) > 1:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path"
            marker.id = 50000
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.5
            
            from geometry_msgs.msg import Point
            for pos in self.cleaning_path[-200:]:  # Last 200 points
                p = Point()
                p.x = pos[0]
                p.y = pos[1]
                p.z = 0.01
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        # 4. Text showing progress
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "stats"
        marker.id = 60000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"Dirty: {len(self.dirty_spots)} | Cleaned: {len(self.cleaned_areas)} | Area: {self.area_cleaned:.1f}m²"
        marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    
    def laser_callback(self, msg):
        """Process laser scan with better obstacle detection"""
        if len(msg.ranges) == 0:
            return
        
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        
        if valid_ranges:
            self.min_distance = min(valid_ranges)
            
            # Front sensor (±30 degrees) - WIDER ANGLE
            front_start = len(msg.ranges) - 30
            front_end = 30
            front_ranges = list(msg.ranges[front_start:]) + list(msg.ranges[:front_end])
            front_valid = [r for r in front_ranges if msg.range_min < r < msg.range_max]
            self.front_distance = min(front_valid) if front_valid else float('inf')
            
            # Left sensor (60-120 degrees)
            left_ranges = msg.ranges[60:120]
            left_valid = [r for r in left_ranges if msg.range_min < r < msg.range_max]
            self.left_distance = min(left_valid) if left_valid else float('inf')
            
            # Right sensor (240-300 degrees)
            right_ranges = msg.ranges[240:300]
            right_valid = [r for r in right_ranges if msg.range_min < r < msg.range_max]
            self.right_distance = min(right_valid) if right_valid else float('inf')
            
            # Wall detection - LEBIH SENSITIF
            self.wall_detected = (self.left_distance < 0.30 or self.right_distance < 0.30)
            
            # Obstacle detection - LEBIH JAUH
            self.obstacle_detected = self.front_distance < 0.30  # Dari 0.3 ke 0.5
            
            # Emergency stop - VERY CLOSE
            self.emergency_stop = self.front_distance < 0.25    
    def odom_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # Add to path
        if len(self.cleaning_path) == 0 or \
           math.sqrt((self.position[0] - self.cleaning_path[-1][0])**2 + 
                    (self.position[1] - self.cleaning_path[-1][1])**2) > 0.1:
            self.cleaning_path.append(self.position)
        
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.check_stuck()
    
    def check_stuck(self):
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
            
            # Reset docking mode when leaving DOCKING state
            if self.current_state == State.DOCKING:
                if hasattr(self, 'docking_mode'):
                    delattr(self, 'docking_mode')
            
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
    
    def distance_to_dock(self):
        dx = self.dock_position[0] - self.position[0]
        dy = self.dock_position[1] - self.position[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def state_idle(self):
        self.publish_velocity(0.0, 0.0)
        msg = Bool()
        msg.data = False
        self.cleaning_pub.publish(msg)
        self.get_logger().info(f'IDLE - Battery: {self.battery_level:.1f}%', throttle_duration_sec=5.0)
        if self.get_elapsed_time() > 3.0 and self.battery_level > self.low_battery_threshold:
            self.get_logger().info('🧹 Starting cleaning!')
            self.change_state(State.CLEANING)
    
    def state_cleaning(self):
        """Main cleaning with better collision avoidance"""
        msg = Bool()
        msg.data = True
        self.cleaning_pub.publish(msg)
        
        # Check and clean spots
        self.check_and_clean_spots()
        
        if self.battery_level < self.low_battery_threshold:
            self.get_logger().warn(f'Low battery: {self.battery_level:.1f}%')
            self.change_state(State.DOCKING)
            return
        
        if self.is_stuck:
            self.get_logger().warn('Robot stuck!')
            self.change_state(State.STUCK)
            return
        
        if self.wall_detected and random.random() < 0.05:
            self.change_state(State.EDGE_CLEANING)
            return
        
        if random.random() < 0.02:
            self.change_state(State.SPOT_CLEANING)
            return
        
        # IMPROVED OBSTACLE AVOIDANCE
        if hasattr(self, 'emergency_stop') and self.emergency_stop:
            # EMERGENCY: Very close! Stop and reverse
            self.publish_velocity(-0.1, 0.0)
            self.get_logger().warn('EMERGENCY STOP!', throttle_duration_sec=1.0)
        elif self.obstacle_detected:
            # Obstacle ahead - smart turning
            # Turn away from closer side
            if self.left_distance < self.right_distance:
                # Turn right (away from left wall)
                self.publish_velocity(0.0, -self.angular_speed)
                self.get_logger().info('Avoiding left', throttle_duration_sec=1.0)
            else:
                # Turn left (away from right wall)
                self.publish_velocity(0.0, self.angular_speed)
                self.get_logger().info('Avoiding right', throttle_duration_sec=1.0)
        elif self.front_distance < 0.8:
            # Getting close - slow down and turn slightly
            turn_direction = 1 if self.left_distance > self.right_distance else -1
            self.publish_velocity(0.1, self.angular_speed * 0.5 * turn_direction)
        else:
            # Safe - normal cleaning
            random_turn = random.uniform(-0.1, 0.1)
            self.publish_velocity(self.cleaning_speed, random_turn)
        
        self.update_battery(1.0)
    
    def state_spot_cleaning(self):
        """Spot cleaning with safety check"""
        msg = Bool()
        msg.data = True
        self.cleaning_pub.publish(msg)
        
        self.check_and_clean_spots()
        
        # SAFETY CHECK
        if self.front_distance < 0.4:
            self.get_logger().warn('Obstacle during spot clean - aborting')
            self.change_state(State.CLEANING)
            return
        
        if self.spiral_radius < 1.5:
            self.spiral_radius += 0.015
            angular_vel = 1.0 / (self.spiral_radius + 0.1)
            self.publish_velocity(self.cleaning_speed * 0.4, angular_vel)
        else:
            self.change_state(State.CLEANING)
        
        self.update_battery(1.5)    

    def state_edge_cleaning(self):
        """Edge cleaning with proper wall following"""
        msg = Bool()
        msg.data = True
        self.cleaning_pub.publish(msg)
        
        self.check_and_clean_spots()
        
        # SAFETY: Stop if too close to front
        if self.front_distance < 0.3:
            self.get_logger().warn('Too close to wall!')
            self.change_state(State.CLEANING)
            return
        
        if self.wall_detected:
            # Follow wall at safe distance
            if self.left_distance < 0.3:
                # Too close to left wall - turn right
                self.publish_velocity(0.1, -0.2)
            elif self.right_distance < 0.3:
                # Too close to right wall - turn left
                self.publish_velocity(0.1, 0.2)
            else:
                # Good distance - follow wall
                self.publish_velocity(0.15, 0.1)
        else:
            # Lost wall
            self.change_state(State.CLEANING)
            return
        
        self.update_battery(1.2)
        
        if self.get_elapsed_time() > 12.0:
            self.change_state(State.CLEANING)
    
    def state_stuck(self):
        msg = Bool()
        msg.data = False
        self.cleaning_pub.publish(msg)
        
        if self.get_elapsed_time() < 1.5:
            self.publish_velocity(-0.15, 0.0)
        elif self.get_elapsed_time() < 3.0:
            self.publish_velocity(0.0, 0.8)
        else:
            self.is_stuck = False
            self.change_state(State.CLEANING)
        
        self.update_battery(0.5)

    def state_docking(self):
        """Simple and reliable docking - GUARANTEED to reach dock"""
        msg = Bool()
        msg.data = False
        self.cleaning_pub.publish(msg)
        
        distance = self.distance_to_dock()
        
        # Arrived!
        if distance < 0.2:  # Increased tolerance
            self.get_logger().info('=== DOCKED SUCCESSFULLY ===')
            self.change_state(State.CHARGING)
            return
        
        # Calculate direction to dock
        dx = self.dock_position[0] - self.position[0]
        dy = self.dock_position[1] - self.position[1]
        target_angle = math.atan2(dy, dx)
        
        # Angle difference
        angle_diff = target_angle - self.yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # SIMPLE STRATEGY: Turn first, then move
        if abs(angle_diff) > 0.2:
            # TURNING MODE: Just turn, don't move forward
            angular_vel = 0.5 if angle_diff > 0 else -0.5
            self.publish_velocity(0.0, angular_vel)
            self.get_logger().info(
                f'🔄 Turning to dock | Angle: {math.degrees(angle_diff):.1f}° | Dist: {distance:.2f}m',
                throttle_duration_sec=1.0
            )
        else:
            # MOVING MODE: Facing dock, now move
            
            # Check for obstacles
            if self.front_distance < 0.4:
                # Obstacle! Stop and turn 90 degrees
                self.get_logger().warn('Obstacle! Turning 90°...')
                self.publish_velocity(0.0, 0.8)
            else:
                # Clear path - move forward slowly
                speed = 0.15
                
                # Slow down when close
                if distance < 1.0:
                    speed = 0.1
                if distance < 0.5:
                    speed = 0.08
                
                # Small angular correction
                correction = angle_diff * 0.5
                
                self.publish_velocity(speed, correction)
                self.get_logger().info(
                    f'➡️  Moving to dock | Dist: {distance:.2f}m | Front: {self.front_distance:.2f}m',
                    throttle_duration_sec=1.0
                )
        
        # Slower battery drain during docking
        self.update_battery(0.2)
    
    def state_charging(self):
        self.publish_velocity(0.0, 0.0)
        msg = Bool()
        msg.data = False
        self.cleaning_pub.publish(msg)
        self.battery_level += 1.0 * 0.1
        self.battery_level = min(100.0, self.battery_level)
        self.get_logger().info(f'Charging... {self.battery_level:.1f}%', throttle_duration_sec=3.0)
        if self.battery_level >= 95.0:
            self.get_logger().info(f'✨ Cleaning complete! Cleaned {len(self.cleaned_areas)} spots, {self.area_cleaned:.1f}m²')
            self.change_state(State.IDLE)
    
    def fsm_loop(self):
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
    robot = CleaningRobotWithViz()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.publish_velocity(0.0, 0.0)
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
