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
        third = len(msg.ranges) // 3
        front_ranges = msg.ranges[third: 2 * third]

        if any(r < self.danger_distance for r in front_ranges if r > 0.05):
            self.get_logger().info('TOO CLOSE TO WALL!')
            self.publish_safety(True)
            self.back_off(msg)
        else:
            self.publish_safety(False)

    def angle_callback(self, msg):
        self.last_ang = msg.data

    def back_off(self, scan_msg):
        # Get valid range readings
        valid_ranges = [(i, r) for i, r in enumerate(scan_msg.ranges) if r > 0.05 and r < scan_msg.range_max]
        if not valid_ranges:
            self.get_logger().warn('No valid LiDAR readings to determine escape angle.')
            return

        # Furthest open direction
        furthest_index, _ = max(valid_ranges, key=lambda x: x[1])
        angle_to_furthest = scan_msg.angle_min + furthest_index * scan_msg.angle_increment

        # Adjust angular speed: exaggerate spin based on how far off-center it is
        angle_center = 0.0  # straight ahead
        angle_diff = angle_to_furthest - angle_center
        angular_bias = 2.0 * angle_diff  # drift harder

        # Motion command
        twist = Twist()
        twist.linear.x = -0.15  # back up slowly
        twist.angular.z = angular_bias  # drift/rotate away

        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Backing off with drift: angular={angular_bias:.2f} rad')

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
