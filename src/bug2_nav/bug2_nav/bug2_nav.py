import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDistanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_distance_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # Filter out 'inf' and 'nan' values
        valid_ranges = [r for r in msg.ranges if r > 0.0 and r < float('inf')]
        if valid_ranges:
            min_distance = min(valid_ranges)
            self.get_logger().info(f'Minimum distance to obstacle: {min_distance:.2f} meters')
        else:
            self.get_logger().info('No valid obstacle detected.')

        # Call the zone analysis
        self.find_obstacle_whereabout(msg)

    def find_obstacle_whereabout(self, msg):
        num_ranges = len(msg.ranges)
        zone_size = num_ranges // 4

        # Handle possible remainder by including all remaining ranges in the 'right' zone
        zones = {
            'front': msg.ranges[0:zone_size],
            'left': msg.ranges[zone_size:2*zone_size],
            'back': msg.ranges[2*zone_size:3*zone_size],
            'right': msg.ranges[3*zone_size:]
        }

        min_distances = {}
        for zone, ranges in zones.items():
            valid = [r for r in ranges if r > 0.0 and r < float('inf')]
            min_distances[zone] = min(valid) if valid else float('inf')

        self.get_logger().info(
            f"Obstacle distances - Front: {min_distances['front']:.2f} m, "
            f"Left: {min_distances['left']:.2f} m, "
            f"Back: {min_distances['back']:.2f} m, "
            f"Right: {min_distances['right']:.2f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()