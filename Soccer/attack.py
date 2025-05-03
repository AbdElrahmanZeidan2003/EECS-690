import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class AttackNode(Node):
    def __init__(self):
        super().__init__('attack_node')
        self.mode = ""
        self.sub_cmd = self.create_subscription(String, '/cmd_strat', self.cmd_callback, 10)
        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.center = 320

    def cmd_callback(self, msg):
        self.mode = msg.data
    def image_callback(self, msg):
        if self.mode != "ATTACK":
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (130, 50, 50), (160, 255, 255))  # purple
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            cx = x + w // 2
            error = self.center - cx
            twist.linear.x = 0.2
            twist.angular.z = -0.002 * error
        else:
            twist.angular.z = 0.3

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AttackNode()
    rclpy.spin(node)
    rclpy.shutdown()
