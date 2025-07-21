#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveTurtleBot(Node):
    def __init__(self):
        super().__init__('move_turtlebot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_callback)

    def move_callback(self):
        move_cmd = Twist()
        
        # Move forward for 3 seconds
        self.get_logger().info("Moving forward...")
        move_cmd.linear.x = 0.2  # Linear velocity (m/s)
        move_cmd.angular.z = 0.0  # No rotation
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self.publisher_.publish(move_cmd)
            time.sleep(0.1)
        
        # Stop the robot
        move_cmd.linear.x = 0.0
        self.publisher_.publish(move_cmd)
        
        # Turn left for 2 seconds
        self.get_logger().info("Turning left...")
        move_cmd.angular.z = 0.5  # Angular velocity (rad/s)
        move_cmd.linear.x = 0.0  # No linear movement
        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.publisher_.publish(move_cmd)
            time.sleep(0.1)
        
        # Stop the robot
        move_cmd.angular.z = 0.0
        self.publisher_.publish(move_cmd)
        
        self.get_logger().info("Movement complete!")
        self.destroy_timer(self.timer)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtleBot()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
