import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
import math

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.min_safe_distance = 0.3
        self.current_ang = 0.0    # Angle from /ang
        self.in_danger = False
        # Subscribers
        self.create_subscription(Float32MultiArray, '/scan_raw', self.scan_callback, 10)
        self.create_subscription(Float32, '/ang', self.ang_callback, 10)
        # Publishers
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    def ang_callback(self, msg):
        self.current_ang = msg.data
    def scan_callback(self, msg):
        distances = msg.data  # [front, right, back, left]
        if len(distances) != 4:
            self.get_logger().warn("Expected 4 distances from scan_raw")
            return
        # Check for danger
        self.in_danger = any(d < self.min_safe_distance for d in distances)
        # Publish /safety 
        safety_msg = Bool()
        safety_msg.data = self.in_danger
        self.safety_pub.publish(safety_msg)
        # Generate response
        twist = Twist()
        if self.in_danger:
            # Pick the farthest direction
            direction_labels = ['front', 'right', 'back', 'left']
            safest_index = distances.index(max(distances))
            safest_dir = direction_labels[safest_index]
            self.get_logger().warn(f"Too close! Escaping toward: {safest_dir}")
            # Default motion for each direction
            if safest_index == 0:  # front
                twist.linear.x = 0.2
            elif safest_index == 1:  # right
                twist.angular.z = -0.5
            elif safest_index == 2:  # back
                twist.linear.x = -0.2
            elif safest_index == 3:  # left
                twist.angular.z = 0.5
            #If robot is close to being aligned with target angle - > stop spinning
            if abs(self.current_ang) < 0.1:
                self.get_logger().info("Aligned with /ang target — stopping!")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
