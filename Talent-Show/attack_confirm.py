import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import Image  
from std_msgs.msg import String  
from geometry_msgs.msg import Twist  
import cv2  
import numpy as np  
from cv_bridge import CvBridge  
import time  

class AttackConfirmNode(Node):
    
    def __init__(self):
        super().__init__('attack_confirm')

        # Subscriber to robot camera feed
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_beep = self.create_publisher(String, '/beep_signal', 10)
        self.publisher_reset = self.create_publisher(String, '/reset_signal', 10)

        self.bridge = CvBridge()
        self.initial_red_cups = []
        self.initial_blue_cups = []
        self.previous_red_cups = []

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  

        # Define color ranges for red and blue cups
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        blue_lower = np.array([100, 120, 70])
        blue_upper = np.array([140, 255, 255])

        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

        red_cups = self.detect_cups(red_mask)
        blue_cups = self.detect_cups(blue_mask)

        if not self.initial_red_cups:
            self.initial_red_cups = red_cups
        if not self.initial_blue_cups:
            self.initial_blue_cups = blue_cups

        if len(red_cups) < len(self.previous_red_cups):
            self.get_logger().info("Hit confirmed on a red cup!")
            self.publisher_beep.publish(String(data="HIT CONFIRM"))

            stop_msg = Twist()
            self.publisher_cmd.publish(stop_msg)
            time.sleep(1.0)

            self.initial_red_cups = red_cups  

            if len(self.initial_red_cups) == 0:
                self.get_logger().info("All targets eliminated! Transitioning to Reset Mode.")
                self.publisher_reset.publish(String(data="START RESET"))

        if len(blue_cups) < len(self.initial_blue_cups):
            self.get_logger().warning("WARNING: Blue cup hit!")
            self.publisher_beep.publish(String(data="ERROR: BLUE CUP HIT"))
            self.initial_blue_cups = blue_cups  

    def detect_cups(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return [(cv2.boundingRect(c)[0], cv2.boundingRect(c)[1]) for c in contours if cv2.contourArea(c) > 500]

def main(args=None):
    rclpy.init(args=args)  
    node = AttackConfirmNode() 
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':
    main()
