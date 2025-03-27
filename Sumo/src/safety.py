import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        # Threshold distance (in meters) to consider dangerous proximity
        self.danger_distance = 0.3
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.ang_sub = self.create_subscription(Float32, '/ang', self.angle_callback, 10)
        self.danger = False
        self.last_ang = 0.0
    def lidar_callback(self, msg: LaserScan):
        # Check if any value in the front sector is below the danger distance
        front_ranges = msg.ranges[len(msg.ranges)//3 : 2*len(msg.ranges)//3]
        if any(r < self.danger_distance for r in front_ranges if r > 0.05):  # ignore invalid readings
            self.danger = True
            self.get_logger().info('Dangerously close to wall!')
            self.publish_safety(True)
            self.back_off()
        else:
            self.danger = False
            self.publish_safety(False)
    def angle_callback(self, msg: Float32):
        self.last_ang = msg.data
    def back_off(self):
        twist = Twist()
        twist.linear.x = -0.3  # Backward
        twist.angular.z = self.last_ang  # Spin away using angle from orange node
        self.cmd_pub.publish(twist)
    def publish_safety(self, flag: bool):
        safety_msg = Bool()
        safety_msg.data = flag
        self.safety_pub.publish(safety_msg)
def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
