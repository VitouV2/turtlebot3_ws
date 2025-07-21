#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class Bug1Node(Node):
    def __init__(self):
        super().__init__('bug1_node')

        # Goal position
        self.goal_x = 11.0
        self.goal_y = 7.0

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0

        # Thresholds
        self.distance_to_goal_threshold = 0.1
        self.wall_detection_threshold = 0.8   # Start reacting earlier
        self.wall_follow_distance = 0.5     # Stay --m from wall

        # Directional zone distances
        self.front_dist = float('inf')       # Wide front region
        self.right_dist = float('inf')       # Right side for wall following

        # State machine
        self.state = "MOVE_TO_GOAL"          # MOVE_TO_GOAL, FOLLOW_WALL
        self.closest_point_on_obstacle = None
        self.closest_distance = float('inf')
        self.boundary_points = []

        # Wall following
        self.wall_follow_side = 'right'

        # Speed settings
        self.forward_speed = 0.3
        self.turning_speed = 0.8

        # Subscribers / Publisher / Timer
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.initialized = False
        self.get_logger().info('Bug1 node initialized.')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.initialized = True

    def scan_callback(self, msg):
        try:
            ranges = msg.ranges

            # Wider front region (to detect walls sooner)
            front_ranges = ranges[315:360] + ranges[0:45]
            valid_front = [r for r in front_ranges if r > 0.01 and r < float('inf')]
            self.front_dist = min(valid_front) if valid_front else float('inf')

            # Right region for wall following
            right_ranges = ranges[270:315]
            valid_right = [r for r in right_ranges if r > 0.01 and r < float('inf')]
            self.right_dist = min(valid_right) if valid_right else float('inf')

        except Exception as e:
            self.get_logger().warn(f"Error in scan callback: {e}")

    def get_direction_to_goal(self):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        desired_angle = math.atan2(dy, dx)
        error = desired_angle - self.yaw
        return math.atan2(math.sin(error), math.cos(error))  # Normalize angle

    def distance_to_point(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

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

            if self.front_dist < self.wall_detection_threshold:
                self.state = "FOLLOW_WALL"
                self.get_logger().info("Wall detected ahead. Starting wall following.")

        elif self.state == "FOLLOW_WALL":
            desired_distance = self.wall_follow_distance
            current_dist = self.right_dist

            # Too close → turn left
            if current_dist < desired_distance / 2:
                twist.angular.z = self.turning_speed
                twist.linear.x = self.forward_speed * 0.5
            # Too far → turn right
            elif current_dist > desired_distance * 1.5:
                twist.angular.z = -self.turning_speed
                twist.linear.x = self.forward_speed * 0.5
            # Just right → go straight
            else:
                twist.angular.z = 0.0
                twist.linear.x = self.forward_speed

            # If clear path ahead, return to goal seeking
            if self.front_dist > self.wall_detection_threshold + 0.5:
                self.get_logger().info("Path is clear. Returning to goal.")
                self.state = "MOVE_TO_GOAL"

        self.cmd_pub.publish(twist)

    def reached_goal(self):
        distance = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        return distance < self.distance_to_goal_threshold


def main(args=None):
    rclpy.init(args=args)
    bug1_node = Bug1Node()
    try:
        rclpy.spin(bug1_node)
    except KeyboardInterrupt:
        bug1_node.get_logger().info("Shutting down Bug1 node.")
    finally:
        bug1_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()