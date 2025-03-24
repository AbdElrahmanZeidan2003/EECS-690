"""
LiDAR → /scan → slam_toolbox (SLAM/localization)
                ↳ localization_node (safety check)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.declare_parameter("min_safe_distance", 0.3)  # meters
        self.min_safe_distance = self.get_parameter("min_safe_distance").value

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg: LaserScan):
        # Check if any distance reading is below the threshold
        too_close = any(
            distance < self.min_safe_distance and distance > 0.05
            for distance in msg.ranges
        )

        if too_close:
            self.get_logger().warn("Too close to wall !!!!!")
        else:
            self.get_logger().info("Safe distance from wall :)")

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
Launch slam toolbox: ros2 launch slam_toolbox localization.py use_sim_time:=false
Run node: ros2 run sth localization
Adjust distance: ros2 run sth localization --ros-args -p min_safe_distance:=0.25

"""


