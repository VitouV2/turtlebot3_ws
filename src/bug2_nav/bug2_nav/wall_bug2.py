#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import math

class SimpleAvoidNode(Node):
    def __init__(self):
        super().__init__('simple_avoid_node')

        # Goal position
        self.goal_x = -11.0
        self.goal_y = 7.0

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0

        # Thresholds
        self.distance_to_goal_threshold = 0.2
        self.obstacle_range_threshold = 1.5  # Start avoiding at 1.5m
        self.min_clearance = 0.8           # Minimum safe distance

        # Sensor readings
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')

        # State machine
        self.state = "MOVE_TO_GOAL"  # MOVE_TO_GOAL, AVOID_OBSTACLE
        self.avoid_direction = None  # left or right

        # Speed settings
        self.forward_speed = 0.2
        self.turning_speed = 0.3

        # Subscribers / Publisher / Timer
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.initialized = False
        self.get_logger().info('Simple Avoid Node initialized.')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.initialized = True

    def scan_callback(self, msg):
        try:
            ranges = msg.ranges

            # Front region for obstacle detection
            front_ranges = [r for r in ranges[315:360] + ranges[0:45] if r > 0.01]
            self.front_dist = min(front_ranges) if front_ranges else float('inf')

            # Left and right for turning decisions
            left_ranges = [r for r in ranges[45:135] if r > 0.01]
            self.left_dist = min(left_ranges) if left_ranges else float('inf')

            right_ranges = [r for r in ranges[225:315] if r > 0.01]
            self.right_dist = min(right_ranges) if right_ranges else float('inf')

        except Exception as e:
            self.get_logger().warn(f"Error in scan callback: {e}")

    def get_direction_to_goal(self):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        desired_angle = math.atan2(dy, dx)
        error = desired_angle - self.yaw
        return math.atan2(math.sin(error), math.cos(error))  # Normalize angle

    def control_loop(self):
        twist = Twist()

        if not self.initialized:
            self.get_logger().info("Waiting for initial data...")
            self.cmd_pub.publish(twist)
            return

        if self.reached_goal():
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Reached goal!")
            self.state = "MOVE_TO_GOAL"
        elif self.state == "MOVE_TO_GOAL":
            angle_to_goal = self.get_direction_to_goal()
            twist.angular.z = max(-0.5, min(0.5, angle_to_goal * 1.5))
            twist.linear.x = self.forward_speed

            if self.front_dist < self.obstacle_range_threshold:
                self.state = "AVOID_OBSTACLE"
                self.get_logger().info("Obstacle ahead. Starting avoidance maneuver.")
                # Decide which way to turn
                if self.left_dist > self.right_dist:
                    self.avoid_direction = "left"
                else:
                    self.avoid_direction = "right"

        elif self.state == "AVOID_OBSTACLE":
            if self.front_dist > self.obstacle_range_threshold + 0.5:
                # Obstacle cleared, return to goal
                self.state = "MOVE_TO_GOAL"
                self.get_logger().info("Obstacle cleared. Resuming goal.")
            else:
                # Turn gently around obstacle
                if self.avoid_direction == "left":
                    twist.angular.z = self.turning_speed
                else:
                    twist.angular.z = -self.turning_speed
                twist.linear.x = self.forward_speed * 0.6  # Slow down during turn

                # Optional: Add soft correction as robot turns
                current_error = self.get_direction_to_goal()
                if abs(current_error) < 0.2:  # Near goal direction
                    self.state = "MOVE_TO_GOAL"
                    self.get_logger().info("Realigning with goal.")

        self.cmd_pub.publish(twist)

    def reached_goal(self):
        distance = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        return distance < self.distance_to_goal_threshold


def main(args=None):
    rclpy.init(args=args)
    node = SimpleAvoidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Simple Avoid Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()