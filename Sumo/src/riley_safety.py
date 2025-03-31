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
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
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
        max_linear = 0.6
        num_scans = len(scan_msg.ranges)
        furthest_idx = scan_msg.ranges.index(max(scan_msg.ranges))

        #if in second quadrant (first quarter), positive x, negative y
        if  0 <= furthest_idx <= (num_scans//4):
            if furthest_idx == 0:
                x_component = 0
            else:
                #second expression equates close to 1 when at front, near 0 when at side
                x_component = max_linear * ((num_scans//4)/furthest_idx)/(num_scans//4)
                y_component = -max_linear * (furthest_idx/(num_scans//4))/(num_scans//4)

        #if in first quadrant (fourth quarter), positive x, positive y
        elif ((num_scans//4)*3 <= furthest_idx <= num_scans):
            if furthest_idx == 0:
                x_component = max_linear
            else:
                section = range((num_scans//4)*3, num_scans+1)
                #second expression equates close to 1 when at front, near 0 when at side
                x_component = max_linear * (((num_scans//4)*3)/furthest_idx)/(num_scans//4)
                y_component = max_linear * (furthest_idx/((num_scans//4)*3))/(num_scans//4)
        
        #if in third quadrant (third quarter), negative x, negative y
        elif ((num_scans//4) <= furthest_idx <= num_scans//2):
            if furthest_idx == num_scans//2:
                x_component = -max_linear
            else:
                #second expression equates close to -1 when at rear, near 0 when at side
                x_component = -max_linear * (((num_scans//2))/furthest_idx)/(num_scans//4)
                y_component = -max_linear * (furthest_idx/(num_scans//4))/(num_scans//4)

        #in fourth quadrant (third quarter), negative x, positive y
        else:
            if furthest_idx == num_scans:
                x_component = max_linear
            else:
                x_component = -max_linear * (((num_scans//2))/furthest_idx)/(num_scans//4)
                y_component = max_linear * (furthest_idx/(num_scans//4)*3)/(num_scans//4)

            


        # Move forward with angular drift
        twist = Twist()
        twist.linear.x = x_component
        twist.linear.y = y_component
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