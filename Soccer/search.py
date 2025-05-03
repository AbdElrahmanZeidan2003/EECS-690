import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class SearchNode(Node):
    def __init__(self):
        super().__init__('search_node')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(String, '/cmd_strat', 10)
        self.bridge = CvBridge()
        self.distance_threshold = 25.0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) +  cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cmd = String()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            ball_dist = 1000 / (w + 1)
            cmd.data = "ATTACK" if ball_dist < self.distance_threshold else "DEFEND"
        else:
            cmd.data = "DEFEND"

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SearchNode()
    rclpy.spin(node)
    rclpy.shutdown()
