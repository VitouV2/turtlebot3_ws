import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, cos, sin, pi
from tf_transformations import euler_from_quaternion  # NEW
import numpy as np

class PotentialFieldController(Node):
    def __init__(self):
        super().__init__('potential_field_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Goal
        self.goal_x = 5.0
        self.goal_y = 4.0

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # in radians

        # Parameters
        self.k_att = 1.5
        self.k_rep = 0.2
        self.repulsion_radius = 0.6

        self.obstacles = []

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Get yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.yaw = euler_from_quaternion(q)  # ✅ Convert to Euler

        # Log position
        self.get_logger().info(f'Position: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad')

        # Compute forces
        force_att = self.compute_attractive_force()
        force_rep = self.compute_repulsive_force()
        force_total = force_att + force_rep

        # Log forces
        self.get_logger().info(f'Attractive: {force_att}, Repulsive: {force_rep}, Total: {force_total}')

        # Control
        twist = Twist()
        distance = np.linalg.norm(force_total)

        if distance > 0.2:
            angle_to_force = atan2(force_total[1], force_total[0])
            angle_error = self.normalize_angle(angle_to_force - self.yaw)

            twist.linear.x = min(0.5, distance)
            twist.angular.z = 2.0 * angle_error
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('🎯 Goal reached!')

        self.publisher_.publish(twist)

    def scan_callback(self, msg):
        self.obstacles.clear()
        angle = msg.angle_min
        count = 0
        for r in msg.ranges:
            if 0.05 < r < self.repulsion_radius:
                obs_x = self.x + r * cos(angle + self.yaw)
                obs_y = self.y + r * sin(angle + self.yaw)
                self.obstacles.append((obs_x, obs_y))
                count += 1
            angle += msg.angle_increment
        self.get_logger().info(f'🚧 Obstacles detected: {count}')

    def compute_attractive_force(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        return np.array([self.k_att * dx, self.k_att * dy])

    def compute_repulsive_force(self):
        force = np.array([0.0, 0.0])
        for ox, oy in self.obstacles:
            dx = self.x - ox
            dy = self.y - oy
            dist = sqrt(dx**2 + dy**2)
            if dist < 0.01:
                continue
            rep_strength = self.k_rep * (1.0 / dist - 1.0 / self.repulsion_radius) / (dist ** 2)
            force += rep_strength * np.array([dx, dy])
        return force

    def normalize_angle(self, angle):
        return (angle + pi) % (2 * pi) - pi

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()