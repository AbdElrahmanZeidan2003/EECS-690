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
        self.orange_seen = False #new

        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.safety_pub = self.create_publisher(Bool, '/safety', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.angle_sub = self.create_subscription(Float32, '/ang', self.angle_callback, 10)
        self.orange_sub = self.create_subscription(Bool, '/orange_seen', self.orange_callback, 10) #new
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
            if self.in_safety_mode and (
                (self.orange_seen and abs(self.last_ang) < 0.5) or
                time.time() - self.last_safety_time > 1.0
            ):
                self.get_logger().info('[SAFETY] Orange reacquired — exiting safety mode')
                self.publish_safety(False)

    def angle_callback(self, msg):
        self.last_ang = msg.data
    
    def orange_callback(self, msg):  #new
        self.orange_seen = msg.data

    def avoid_obstacle(self, scan_msg):
        def angle_to_index(angle_rad):
            return max(0, min(len(scan_msg.ranges) - 1,
                              int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)))

        # Explicit directional strategy
        directions = {
            'front': angle_to_index(0.0),
            'left': angle_to_index(math.radians(90)),
            'right': angle_to_index(math.radians(-90)),
            'back': angle_to_index(math.radians(180)) if scan_msg.angle_max > math.pi else angle_to_index(math.radians(-180))
        }

        distances = {}
        for name, i in directions.items():
            d = scan_msg.ranges[i]
            distances[name] = d if 0.05 < d < scan_msg.range_max else float('inf')

        closest_dir, closest_dist = min(distances.items(), key=lambda x: x[1])

        twist = Twist()

        if closest_dir == 'front':
            twist.linear.x = -0.15
            twist.angular.z = 1.5
            self.get_logger().info('[ESCAPE] Danger in FRONT — backing up and turning')
        elif closest_dir == 'left':
            twist.linear.x = 0.0
            twist.angular.z = -1.5
            self.get_logger().info('[ESCAPE] Danger on LEFT — turning right')
        elif closest_dir == 'right':
            twist.linear.x = 0.0
            twist.angular.z = 1.5
            self.get_logger().info('[ESCAPE] Danger on RIGHT — turning left')
        elif closest_dir == 'back':
            twist.linear.x = 0.15
            twist.angular.z = 0.0
            self.get_logger().info('[ESCAPE] Danger in BACK — moving forward')

        self.cmd_pub.publish(twist)

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
