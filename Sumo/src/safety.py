import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.min_safe_distance = 0.3
        self.target_angle = None
        self.current_yaw = 0.0  
        # Subscriptions
        self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 10)
        self.create_subscription(Float32, '/ang', self.angle_callback, 10)
        # Publishers
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vcl', 10)
    def angle_callback(self, msg):
        self.target_angle = msg.data
    def scan_callback(self, msg: LaserScan):
        distances = msg.ranges
        if len(distances) < 4:
            self.get_logger().warn("Expected 4 distances from /scan_raw")
            return
        too_close = any(0.05 < d < self.min_safe_distance for d in distances)
        safety_msg = Bool()
        safety_msg.data = too_close
        self.safety_pub.publish(safety_msg)
        twist = Twist()
        if too_close:
            #choose farthest direction
            max_index = max(range(4), key=lambda i: distances[i])
            direction_labels = ["front", "right", "back", "left"]
            self.get_logger().info(f"Too close! Escaping to: {direction_labels[max_index]}")
            #escape logic
            if max_index == 0:
                twist.linear.x = 0.2
            elif max_index == 1:
                twist.angular.z = -1.0
            elif max_index == 2:
                twist.linear.x = -0.2
            elif max_index == 3:
                twist.angular.z = 1.0
            # Stop if aligned with /ang
            if self.target_angle is not None:
                # Simulate current yaw for now 
                current = self.current_yaw
                error = self.angle_diff(self.target_angle, current)
                if abs(error) < 0.1:  # within ~6 degrees
                    self.get_logger().info("Aligned with attack angle — stopping escape.")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
    def angle_diff(self, a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
