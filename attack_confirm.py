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

    # AttackConfirmNode: Monitors attack success by detecting whether the red cups disappear and ensuring no blue cups are hit
    # Publishes confirmation beeps and hit validation
    
    def __init__(self):
        super().__init__('attack_confirm')

        # Subscriber to robot camera feed
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Publisher for movement commands to /cmd_vel
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for beep signals upon hit confirmation
        self.publisher_beep = self.create_publisher(String, '/beep_signal', 10)

        # OpenCV bridge for converting ROS2 images
        self.bridge = CvBridge()

        # Initial list of detected red and blue cups
        self.initial_red_cups = []
        self.initial_blue_cups = []

    def image_callback(self, msg):

        #Callback function that processes incoming camera images, detects red & blue cups, and verifies attack success
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert ROS2 image message to OpenCV format
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV color space

        # Define color ranges for red and blue cups
        red_lower = np.array([0, 120, 70])
        red_upper = np.array([10, 255, 255])
        blue_lower = np.array([100, 120, 70])
        blue_upper = np.array([140, 255, 255])

        # Create masks to detect red and blue objects
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)

        # Get contours for red and blue objects
        red_cups = self.detect_cups(red_mask)
        blue_cups = self.detect_cups(blue_mask)

        # If this is the first frame, store the initial state
        if not self.initial_red_cups:
            self.initial_red_cups = red_cups
        if not self.initial_blue_cups:
            self.initial_blue_cups = blue_cups

        # Check if a red cup has disappeared (successful hit)
        if len(red_cups) < len(self.initial_red_cups):
            self.get_logger().info("Hit confirmed on a red cup!")
            self.publisher_beep.publish(String(data="HIT CONFIRM"))
            self.initial_red_cups = red_cups  # Update reference list

        # Check if a blue cup has been hit (error)
        if len(blue_cups) < len(self.initial_blue_cups):
            self.get_logger().warning("WARNING: Blue cup hit!")
            self.publisher_beep.publish(String(data="ERROR: BLUE CUP HIT"))
            self.initial_blue_cups = blue_cups  # Update reference list

        # Display debugging view (optional)
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def detect_cups(self, mask):
        #Detects objects (cups) based on a binary mask and returns their coordinates.
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cup_positions = []

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filter out small objects
                x, y, w, h = cv2.boundingRect(contour)
                cup_positions.append((x + w // 2, y + h // 2))  # Store center coordinates
        
        return cup_positions

def main(args=None):

    #Main function to initialize and run the attack confirmation node.

    rclpy.init(args=args)  
    node = AttackConfirmNode() 
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':
    main()  
