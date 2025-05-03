import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
from math import sqrt

def calc_rotation(x, resolution=640):
    dcenter = x - resolution / 2
    if abs(dcenter) <= resolution / 5:
        return 0.0 if abs(dcenter) <= 15 else sqrt(abs(dcenter)) / 16 * 2.0
    return -2.0 * (sqrt(abs(dcenter)) / 8)

class DefendNode(Node):
    def __init__(self):
        super().__init__('defend_node')
        self.bridge = CvBridge()
        self.mode = ""
        self.sub_cmd = self.create_subscription(String, '/cmd_strat', self.cmd_callback, 10)
        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ang_pub = self.create_publisher(Float32, '/ang', 10)

    def cmd_callback(self, msg):
        self.mode = msg.data

    def image_callback(self, msg):
        if self.mode != "DEFEND":
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([45, 100, 100])
        upper_green = np.array([75, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, _, w, _ = cv2.boundingRect(largest)
            cx = x + w // 2
            twist.linear.x = -2.0
            twist.angular.z = calc_rotation(cx)
        self.cmd_pub.publish(twist)
        self.ang_pub.publish(Float32(data=twist.angular.z))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DefendNode())
    rclpy.shutdown()
