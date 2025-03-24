"""
LiDAR → /scan → slam_toolbox (SLAM/localization)
                ↳ localization_node (safety check)

Launch slam toolbox: ros2 launch slam_toolbox localization.py use_sim_time:=false
Run node: ros2 run sth localization
Adjust distance: ros2 run sth localization --ros-args -p min_safe_distance:=0.25
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.declare_parameter("min_safe_distance", 0.3)  # meters
        self.min_safe_distance = self.get_parameter("min_safe_distance").value
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # LiDAR subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.backing_up = False

    def scan_callback(self, msg: LaserScan):
        # Check if robot is too close to wall
        too_close = any(
            distance < self.min_safe_distance and distance > 0.05
            for distance in msg.ranges
        )
        if too_close and not self.backing_up:
            self.get_logger().warn("Too close to wall!!!!!")
            self.backing_up = True
            self.reverse_robot(duration=1.5)
            self.backing_up = False
        else:
            self.get_logger().info("Safe distance from wall :)")

# Emergency backup plan when it's too close to the wall 
    def reverse_robot(self, duration=1.5):
        # Create backward Twist message
        twist = Twist()
        twist.linear.x = -0.2  # Move backward
        twist.angular.z = 0.0
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while (self.get_clock().now().seconds_nanoseconds()[0] - start_time) < duration:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
        # Stop the robot after reversing
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
