import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.danger_distance = 0.3
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.angle_sub = self.create_subscription(Float32, '/ang', self.angle_callback, 10)
        self.last_ang = 0.0

    def lidar_callback(self, msg):
        n = len(msg.ranges)
        front = msg.ranges[n//8 : 3*n//8]
        left = msg.ranges[3*n//8 : 5*n//8]
        rear = msg.ranges[5*n//8 : 7*n//8]
        right = msg.ranges[7*n//8:] + msg.ranges[:n//8]

        # Monitor different zones
        zones = {'front': front, 'left': left, 'rear': rear, 'right': right}
        min_dists = {k: min([r for r in v if r > 0.05], default=10.0) for k, v in zones.items()}
        self.get_logger().info(f"Min distances: {min_dists}")

        if min_dists['front'] < self.danger_distance:
            self.get_logger().info('FRONT WALL DANGER!')
            self.publish_safety(True)
            self.avoid_wall(msg)
        else:
            self.publish_safety(False)

    def angle_callback(self, msg):
        self.last_ang = msg.data

    def avoid_wall(self, scan_msg):
        valid_ranges = [(i, r) for i, r in enumerate(scan_msg.ranges) if r > 0.05 and r < scan_msg.range_max]
        if not valid_ranges:
            self.get_logger().warn('No valid LiDAR readings to determine avoidance angle.')
            return

        n = len(scan_msg.ranges)
        center_index = n // 2

        # Only consider angles within +/- 90 degrees of forward (exclude behind)
        escape_candidates = [(i, r) for i, r in valid_ranges if abs(i - center_index) < n // 4]

        if not escape_candidates:
            self.get_logger().warn('No safe forward-ish escape path found.')
            return

        furthest_index, _ = max(escape_candidates, key=lambda x: x[1])
        angle_to_furthest = scan_msg.angle_min + furthest_index * scan_msg.angle_increment

        # Combine with last_ang from vision
        combined_angular = 0.7 * angle_to_furthest + 0.3 * self.last_ang

        twist = Twist()
        twist.linear.x = 0.1  # move forward slowly while avoiding
        twist.angular.z = combined_angular

        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Avoiding wall with forward drift: angular={combined_angular:.2f} rad')

    def publish_safety(self, is_danger):
        msg = Bool()
        msg.data = is_danger
        self.safety_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
