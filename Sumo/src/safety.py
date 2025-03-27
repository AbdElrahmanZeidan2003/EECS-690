import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.danger_distance = 0.3
        self.last_ang = 0.0
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.angle_sub = self.create_subscription(Float32, '/ang', self.angle_callback, 10)

    def lidar_callback(self, msg):
        # Find closest valid range and its angle
        valid_ranges = [(i, r) for i, r in enumerate(msg.ranges) if r > 0.05 and r < msg.range_max]
        if not valid_ranges:
            self.publish_safety(False)
            return

        closest_index, closest_range = min(valid_ranges, key=lambda x: x[1])
        if closest_range < self.danger_distance:
            self.get_logger().info(f'TOO CLOSE TO WALL! Closest at index {closest_index} = {closest_range:.2f} m')
            self.publish_safety(True)
            self.avoid_obstacle(msg)
        else:
            self.publish_safety(False)

    def angle_callback(self, msg):
        self.last_ang = msg.data

    def avoid_obstacle(self, scan_msg):
        # Define index ranges for back, left, right sectors
        sectors = {
            'back': list(range(249, 252)),
            'left': [127],
            'right': list(range(380, 391))
        }

        sector_max = {}
        for name, indices in sectors.items():
            valid_readings = [(i, scan_msg.ranges[i]) for i in indices if scan_msg.ranges[i] > 0.05 and scan_msg.ranges[i] < scan_msg.range_max]
            if valid_readings:
                max_i, max_r = max(valid_readings, key=lambda x: x[1])
                angle = scan_msg.angle_min + max_i * scan_msg.angle_increment
                sector_max[name] = (angle, max_r)

        # Pick the sector with the furthest safe reading
        if sector_max:
            best_sector = max(sector_max.items(), key=lambda x: x[1][1])
            best_angle = best_sector[1][0]
            self.get_logger().info(f"Escaping toward {best_sector[0]} at angle {best_angle:.2f} rad")
        else:
            self.get_logger().warn("No valid escape direction found in defined sectors.")
            return

        # Move forward with angular drift
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = best_angle
        self.cmd_pub.publish(twist)

        # Stop safety mode if target is aligned
        if abs(self.last_ang) < 0.2:
            self.get_logger().info('Target aligned again — exiting safety mode')
            self.publish_safety(False)

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
