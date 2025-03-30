import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
import time
import math

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.danger_distance = 0.3
        self.last_ang = 0.0
        self.in_safety_mode = False
        self.last_safety_time = 0.0  # Cooldown timer

        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.angle_sub = self.create_subscription(Float32, '/ang', self.angle_callback, 10)

    def lidar_callback(self, msg):
        valid_ranges = [(i, r) for i, r in enumerate(msg.ranges) if 0.05 < r < msg.range_max]
        if not valid_ranges:
            self.publish_safety(False)
            return

        _, closest_range = min(valid_ranges, key=lambda x: x[1])
        if closest_range < self.danger_distance:
            self.get_logger().info(f'TOO CLOSE TO WALL! Closest = {closest_range:.2f} m')
            self.publish_safety(True)
            self.avoid_obstacle(msg)
        else:
            if self.in_safety_mode and time.time() - self.last_safety_time > 1.0 and abs(self.last_ang) < 0.2:
                self.get_logger().info('Target aligned — exiting safety mode')
                self.publish_safety(False)

    def angle_callback(self, msg):
        self.last_ang = msg.data

    def avoid_obstacle(self, scan_msg):
        def angle_to_index(angle_rad):
            return max(0, min(len(scan_msg.ranges) - 1,
                              int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)))

        # HYBRID: Wider sector coverage but still simple
        sectors = {
            'back': list(range(angle_to_index(math.radians(130)), angle_to_index(math.radians(-130)))),
            'left': list(range(angle_to_index(math.radians(60)), angle_to_index(math.radians(120)))),
            'right': list(range(angle_to_index(math.radians(-120)), angle_to_index(math.radians(-60))))
        }

        sector_max = {}
        for name, indices in sectors.items():
            valid_readings = [(i, scan_msg.ranges[i]) for i in indices
                              if 0.05 < scan_msg.ranges[i] < scan_msg.range_max]
            if valid_readings:
                max_i, max_r = max(valid_readings, key=lambda x: x[1])
                angle = scan_msg.angle_min + max_i * scan_msg.angle_increment
                sector_max[name] = (angle, max_r)

        if sector_max:
            best_sector = max(sector_max.items(), key=lambda x: x[1][1])
            best_angle = best_sector[1][0]
            self.get_logger().info(f"[SAFETY] Escaping toward {best_sector[0]} at {best_angle:.2f} rad")

            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = best_angle
            self.cmd_pub.publish(twist)
        else:
            self.get_logger().warn("[SAFETY] No valid escape direction found.")

    def publish_safety(self, is_danger):
        if is_danger != self.in_safety_mode:
            self.in_safety_mode = is_danger
            msg = Bool()
            msg.data = is_danger
            self.safety_pub.publish(msg)
            if is_danger:
                self.last_safety_time = time.time()
                self.get_logger().info("[SAFETY] Entering safety mode")
            else:
                self.get_logger().info("[SAFETY] Exiting safety mode")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
