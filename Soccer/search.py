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
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(String, '/cmd_strat', 10)
        self.distance_threshold = 25.0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #Need to adjust this 
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cmd = String()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            _, _, w, _ = cv2.boundingRect(largest)
            distance = 1000 / (w + 1)
            cmd.data = "ATTACK" if distance < self.distance_threshold else "DEFEND"
        else:
            cmd.data = "DEFEND"
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SearchNode())
    rclpy.shutdown()
