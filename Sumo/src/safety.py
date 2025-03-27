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
        # Check center 1/3 of the scan for close obstacles
        third = len(msg.ranges) // 3
        front_ranges = msg.ranges[third: 2*third]
        if any(r < self.danger_distance for r in front_ranges if r > 0.05):
            self.get_logger().info('TOO CLOSE TO WALL!')
            self.publish_safety(True)
            self.back_off()
        else:
            self.publish_safety(False)

    def angle_callback(self, msg):
        self.last_ang = msg.data
    def back_off(self):
        twist = Twist()
        twist.linear.x = -0.2   # move backward
        twist.angular.z = self.last_ang  # spin based on latest orange angle
        self.cmd_pub.publish(twist)

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
