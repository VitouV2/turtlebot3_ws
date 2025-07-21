#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class OrientedWallFollower(Node):
    def __init__(self):
        super().__init__('oriented_wall_follower')

        # Subscribers and Publishers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Configurable Parameters
        self.wall_distance = 1.0     # Distance from wall in meters
        self.wall_side = 'right'     # Can be 'left' or 'right'
        self.forward_speed = 0.2
        self.turn_correction_gain = 1.0  # How aggressively to align with wall

        # Angle ranges for wall detection (in degrees)
        if self.wall_side == 'right':
            self.wall_angles_deg = list(range(-70, 20))  # Right side
        else:
            self.wall_angles_deg = list(range(160, 250))  # Left side

        # Internal state
        self.ranges = []

        self.get_logger().info(f"Wall follower started — following wall on {self.wall_side} side")

    def scan_callback(self, msg):
        self.ranges = msg.ranges

    def get_wall_points(self):
        """Returns list of (angle_rad, distance) pairs from wall-side angles"""
        if not self.ranges:
            return []

        angle_min = -math.pi / 2  # LIDAR starts at -90 deg
        angle_increment = (math.pi * 2) / len(self.ranges)  # Assume 360 deg scan

        wall_points = []
        for deg in self.wall_angles_deg:
            rad = math.radians(deg)
            index = int((rad - angle_min) / angle_increment)
            if 0 <= index < len(self.ranges):
                dist = self.ranges[index]
                if 0.1 < dist < 5.0:  # Filter invalid values
                    wall_points.append((rad, dist))

        return wall_points

    def polar_to_cartesian(self, angle, distance):
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

    def control_loop(self):
        twist = Twist()

        # Get wall data
        wall_points = self.get_wall_points()
        if not wall_points:
            # No wall detected → turn gently toward wall side to find it
            if self.wall_side == 'right':
                twist.angular.z = -0.3  # Turn right
            else:
                twist.angular.z = 0.3   # Turn left
            self.get_logger().info("Searching for wall...")
            self.cmd_pub.publish(twist)
            return

        # Convert to Cartesian coordinates relative to robot
        cart_points = [self.polar_to_cartesian(rad, d) for rad, d in wall_points]

        # Fit a line to the wall points (Ax + By + C = 0)
        xs = np.array([p[0] for p in cart_points])
        ys = np.array([p[1] for p in cart_points])
        A, B, C = self.fit_line(xs, ys)

        if A == 0 and B == 0:
            self.get_logger().warn("Line fit failed.")
            return

        # Wall normal angle (angle of the wall relative to robot)
        wall_angle = math.atan2(B, A)

        # Compute desired robot orientation to be parallel to wall
        if self.wall_side == 'right':
            desired_yaw = wall_angle - math.radians(90)
        else:
            desired_yaw = wall_angle + math.radians(90)

        # Normalize desired yaw
        desired_yaw = math.atan2(math.sin(desired_yaw), math.cos(desired_yaw))

        # Compute error between current heading and desired heading
        yaw_error = desired_yaw

        # Proportional control for turning
        twist.angular.z = self.turn_correction_gain * yaw_error
        twist.linear.x = self.forward_speed

        # Optional: Maintain fixed distance from wall
        avg_dist = sum(d for _, d in wall_points) / len(wall_points)
        desired_dist = self.wall_distance
        if self.wall_side == 'right':
            if avg_dist < desired_dist:
                twist.angular.z -= 0.2  # Too close → turn away
            elif avg_dist > desired_dist:
                twist.angular.z += 0.2  # Too far → turn closer
        else:
            if avg_dist < desired_dist:
                twist.angular.z += 0.2
            elif avg_dist > desired_dist:
                twist.angular.z -= 0.2

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Wall aligned — Avg Dist: {avg_dist:.2f} m | Yaw Error: {math.degrees(yaw_error):.2f}°")

    def fit_line(self, xs, ys):
        """Fit Ax + By + C = 0 line to points using least squares"""
        n = len(xs)
        if n < 2:
            return 0, 0, 0
        mean_x = np.mean(xs)
        mean_y = np.mean(ys)
        u = xs - mean_x
        v = ys - mean_y
        Sxx = np.sum(u * u)
        Syy = np.sum(v * v)
        Sxy = np.sum(u * v)
        if abs(Sxx - Syy) < 1e-6 and abs(Sxy) < 1e-6:
            return 0, 0, 0  # All points are colinear
        eigval, eigvec = np.linalg.eig(np.array([[Sxx, Sxy], [Sxy, Syy]]))
        idx = np.argmin(eigval)
        a, b = eigvec[:, idx]
        c = -a * mean_x - b * mean_y
        return a, b, c


def main(args=None):
    rclpy.init(args=args)
    node = OrientedWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()