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
        self.last_safety_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)

        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.angle_sub = self.create_subscription(Float32, '/ang', self.angle_callback, 10)

    def lidar_callback(self, msg):
        valid_ranges = [(i, r) for i, r in enumerate(msg.ranges) if r > 0.05 and r < msg.range_max]
        if not valid_ranges:
            self.set_safety_state(False)
            return

        _, closest_range = min(valid_ranges, key=lambda x: x[1])

        if closest_range < self.danger_distance:
            self.get_logger().info(f'[SAFETY] Too close! Closest = {closest_range:.2f} m')
            self.set_safety_state(True)
            self.avoid_obstacle(msg)
        else:
            if self.in_safety_mode:
                # Wait at least 1.5 seconds before turning off safety mode
                if time.time() - self.last_safety_time > 1.5 and abs(self.last_ang) < 0.2:
                    self.get_logger().info('[SAFETY] Target re-aligned — exiting safety mode')
                    self.set_safety_state(False)

    def angle_callback(self, msg):
        self.last_ang = msg.data

    def avoid_obstacle(self, scan_msg):
        def angle_to_index(angle_rad):
            index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
            return max(0, min(index, len(scan_msg.ranges) - 1))

        sectors = {
            'back': [angle_to_index(math.radians(135)), angle_to_index(math.radians(-135))],
            'left': [angle_to_index(math.radians(90))],
            'right': [angle_to_index(math.radians(-90))]
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
            self.get_logger().info(f'[SAFETY] Escaping toward {best_sector[0]} (angle {best_angle:.2f} rad)')
        else:
            self.get_logger().warn('[SAFETY] No valid escape direction found!')
            return

        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = best_angle
        self.cmd_pub.publish(twist)

    def set_safety_state(self, new_state):
        if new_state != self.in_safety_mode:
            self.in_safety_mode = new_state
            msg = Bool()
            msg.data = new_state
            self.safety_pub.publish(msg)
            if new_state:
                self.last_safety_time = time.time()
                self.get_logger().info('[SAFETY] Entering safety mode')
            else:
                self.get_logger().info('[SAFETY] Exiting safety mode')

    def publish_safety(self, is_danger):  # kept for compatibility
        self.set_safety_state(is_danger)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

