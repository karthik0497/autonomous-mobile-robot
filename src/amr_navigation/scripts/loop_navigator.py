#!/usr/bin/env python3
"""
Autonomous Warehouse Loop Navigator with Real-Time LiDAR Obstacle Avoidance
Continuously navigates through warehouse aisles in an endless loop around
the work table, chairs, storage racks, and pallets without colliding with anything.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WarehouseLoopNavigator(Node):
    def __init__(self):
        super().__init__('warehouse_loop_navigator')

        # Navigation and obstacle avoidance parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.85)
        self.declare_parameter('safety_distance', 0.65)      # Slow down & steer away
        self.declare_parameter('critical_distance', 0.42)    # Stop forward motion & turn
        self.declare_parameter('waypoint_radius', 0.55)

        self.max_v = float(self.get_parameter('max_linear_speed').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.safety_dist = float(self.get_parameter('safety_distance').value)
        self.critical_dist = float(self.get_parameter('critical_distance').value)
        self.wp_radius = float(self.get_parameter('waypoint_radius').value)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriptions
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, sensor_qos
        )
        
        # Primary: EKF filtered odometry. Fallback: Raw wheel odometry
        self.filtered_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.filtered_odom_callback, 10
        )
        self.raw_odom_sub = self.create_subscription(
            Odometry, '/odom', self.raw_odom_callback, 10
        )

        # Robot State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.has_odom = False
        self.using_ekf = False

        # LiDAR Obstacle Analysis
        self.latest_scan = None
        self.min_front_dist = float('inf')
        self.min_left_dist = float('inf')
        self.min_right_dist = float('inf')
        self.min_overall_dist = float('inf')
        self.repulsive_vy = 0.0

        # Warehouse Loop Waypoints (Counter-Clockwise Circuit)
        # Perfectly placed in the open aisles around table, chairs, racks, and pallets
        self.waypoints = [
            ( 1.5,  1.8),   # WP 0: North aisle past table & chair
            ( 3.5,  1.5),   # WP 1: North-East corner open corridor
            ( 3.5, -1.5),   # WP 2: East aisle past work table
            ( 1.5, -1.8),   # WP 3: South aisle past table & chair
            ( 0.0, -1.8),   # WP 4: South open aisle
            (-3.5,  0.0),   # WP 5: West highway corridor behind storage racks
            ( 0.0,  1.8),   # WP 6: North return aisle
        ]
        self.current_wp_idx = 0

        # Heartbeat timer for diagnostics (every 1.0 second)
        self.log_counter = 0

        # Control loop at 20 Hz (every 50 ms)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('=' * 60)
        self.get_logger().info('Warehouse Loop Navigator INITIALIZED!')
        self.get_logger().info('Patrolling 7 warehouse waypoints with LiDAR collision avoidance.')
        self.get_logger().info('=' * 60)

    def filtered_odom_callback(self, msg: Odometry):
        self._update_pose(msg)
        self.using_ekf = True
        self.has_odom = True

    def raw_odom_callback(self, msg: Odometry):
        if not self.using_ekf:
            self._update_pose(msg)
            self.has_odom = True

    def _update_pose(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y

        # Quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges[:len(angles)])

        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & ~np.isnan(ranges) & ~np.isinf(ranges)
        if not np.any(valid_mask):
            self.min_front_dist = float('inf')
            self.min_left_dist = float('inf')
            self.min_right_dist = float('inf')
            self.min_overall_dist = float('inf')
            self.repulsive_vy = 0.0
            return

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        self.min_overall_dist = float(np.min(valid_ranges))

        # Sector breakdown
        front_mask = np.abs(valid_angles) < math.radians(35)
        left_mask = (valid_angles >= math.radians(35)) & (valid_angles <= math.radians(90))
        right_mask = (valid_angles <= -math.radians(35)) & (valid_angles >= -math.radians(90))

        self.min_front_dist = float(np.min(valid_ranges[front_mask])) if np.any(front_mask) else float('inf')
        self.min_left_dist = float(np.min(valid_ranges[left_mask])) if np.any(left_mask) else float('inf')
        self.min_right_dist = float(np.min(valid_ranges[right_mask])) if np.any(right_mask) else float('inf')

        # Repulsive force in robot lateral axis (Y) from nearby obstacles
        rep_y = 0.0
        for r, a in zip(valid_ranges, valid_angles):
            if r < self.safety_dist:
                force = (1.0 / (r * r)) - (1.0 / (self.safety_dist * self.safety_dist))
                rep_y -= force * math.sin(a)

        self.repulsive_vy = rep_y

    def control_loop(self):
        # Must have odometry and laser scan to navigate
        if not self.has_odom:
            if self.log_counter % 20 == 0:
                self.get_logger().warn('Waiting for odometry topic (/odometry/filtered or /odom)...')
            self.log_counter += 1
            return

        if self.latest_scan is None:
            if self.log_counter % 20 == 0:
                self.get_logger().warn('Waiting for LiDAR topic (/scan)...')
            self.log_counter += 1
            return

        target_x, target_y = self.waypoints[self.current_wp_idx]
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        dist_to_wp = math.hypot(dx, dy)

        # Check if waypoint reached -> Advance to next waypoint in loop
        if dist_to_wp < self.wp_radius:
            old_idx = self.current_wp_idx
            self.current_wp_idx = (self.current_wp_idx + 1) % len(self.waypoints)
            next_x, next_y = self.waypoints[self.current_wp_idx]
            self.get_logger().info(
                f'🎯 Reached WP {old_idx}! Next -> WP {self.current_wp_idx} at ({next_x:.1f}, {next_y:.1f})'
            )
            return

        # Heading error towards target waypoint
        target_angle_world = math.atan2(dy, dx)
        heading_error = target_angle_world - self.current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Attractive baseline commands
        # Allow forward motion when heading error is within ~70 deg
        cos_error = max(0.0, math.cos(heading_error))
        att_v = self.max_v * (cos_error ** 1.5)
        att_w = float(np.clip(1.5 * heading_error, -self.max_w, self.max_w))

        cmd = Twist()

        # Dynamic LiDAR Collision Avoidance
        if self.min_front_dist < self.critical_dist:
            # Emergency turn: Obstacle directly ahead inside critical boundary
            cmd.linear.x = 0.0
            # Turn towards whichever side has more clearance
            turn_dir = 1.0 if self.min_left_dist >= self.min_right_dist else -1.0
            cmd.angular.z = turn_dir * self.max_w
        elif self.min_front_dist < self.safety_dist:
            # Proximity warning: Smoothly reduce speed and blend repulsive steering
            slowdown = (self.min_front_dist - self.critical_dist) / (self.safety_dist - self.critical_dist)
            cmd.linear.x = float(np.clip(att_v * slowdown, 0.05, self.max_v))
            rep_steer = 0.5 * self.repulsive_vy
            cmd.angular.z = float(np.clip(att_w + rep_steer, -self.max_w, self.max_w))
        else:
            # Clear corridor: Drive smoothly to waypoint with minimum forward drive
            cmd.linear.x = float(np.clip(max(att_v, 0.12), 0.10, self.max_v))
            cmd.angular.z = att_w

        self.cmd_pub.publish(cmd)

        # Periodic status heartbeat every 1 second (20 cycles)
        self.log_counter += 1
        if self.log_counter % 20 == 0:
            odom_src = "EKF" if self.using_ekf else "Wheel"
            self.get_logger().info(
                f'🚗 [WP {self.current_wp_idx} ({target_x:.1f},{target_y:.1f})] '
                f'Dist: {dist_to_wp:.2f}m | '
                f'Pos: ({self.current_x:.2f},{self.current_y:.2f}) | '
                f'Cmd: v={cmd.linear.x:.2f} m/s, w={cmd.angular.z:.2f} rad/s | '
                f'Laser min: {self.min_overall_dist:.2f}m ({odom_src})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseLoopNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
