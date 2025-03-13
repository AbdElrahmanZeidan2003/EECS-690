import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

class DefenseNode(Node):
    def __init__(self):
        super().__init__('defense_node')

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.image_sub = self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)

        # Utilities
        self.bridge = CvBridge()
        self.cmd = Twist()

        # Parameters
        self.edge_distance = 0.3  # meters, maintain distance from walls
        self.defense_active = True

    def lidar_callback(self, scan_data):
        min_distance = min(scan_data.ranges)

        # Drifting along edges
        if min_distance < self.edge_distance:
            self.get_logger().info('Near edge, drifting along.')
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.5  # Drift along the edge
            self.vel_pub.publish(self.cmd)

    def image_callback(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([25, 255, 255])

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            self.get_logger().info('Opponent attack detected, dodging.')
            self.cmd.linear.x = -0.2
            self.cmd.angular.z = 0.6  # Quick turn to dodge
            self.vel_pub.publish(self.cmd)
            self.defense_active = True
        else:
            self.defense_active = False

    def is_defense_active(self):
        return self.defense_active


def main(args=None):
    rclpy.init(args=args)
    defense_node = DefenseNode()

    rclpy.spin(defense_node)

    defense_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
