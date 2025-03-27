import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        # Proximity threshold (in meters)
        self.danger_distance = 0.3
        self.last_ang = 0.0
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)
        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.angle_sub = self.create_subscription(Float32, '/ang', self.angle_callback, 10)
    def lidar_callback(self, msg):
        third = len(msg.ranges) // 3
        front_ranges = msg.ranges[third: 2*third]
        # Check if too close in the front third
        if any(r < self.danger_distance for r in front_ranges if r > 0.05):
            self.get_logger().info('TOO CLOSE TO WALL!')
            self.publish_safety(True)
            self.back_off(msg)
        else:
            self.publish_safety(False)
    def angle_callback(self, msg):
        self.last_ang = msg.data
    def back_off(self, scan_msg):
        # Find furthest point from LiDAR scan
        valid_ranges = [(i, r) for i, r in enumerate(scan_msg.ranges) if r > 0.05 and r < scan_msg.range_max]
        if not valid_ranges:
            self.get_logger().warn('No valid LiDAR readings to determine escape angle.')
            return
        furthest_index, _ = max(valid_ranges, key=lambda x: x[1])
        angle_to_furthest = scan_msg.angle_min + furthest_index * scan_msg.angle_increment
        # Back and rotate toward open space
        twist = Twist()
        twist.linear.x = -0.2
        twist.angular.z = angle_to_furthest
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Backing off and spinning toward open angle: {angle_to_furthest:.2f} rad')
        
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
