import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
import cv2
from cv_bridge import cv_bridge
import numpy as np

class AttackNode(Node):
    def __init__(self):
        super().__init__('attack_node')
        #publishing 
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ang_pub = self.create_publisher(Float32, '/ang', 10)
        #subing 
        self.image_sub = self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image', self.image_callback, 10)
        self.safety_sub = self.create_subscription(Bool, '/safety', self.safety_callback, 10)

        self.bridge = CvBridge()
    def image_callback(self, msg):
        try:
            # ROS2 image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # ORANGE COLOR MASK RANGE
        lower_orange = np.array([0, 165, 104])
        upper_orange = np.array([255, 255, 255])
        
        # threshold the image to get only orange pixels
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        
        # Find contours of the orange regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour based on area
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # bounding box of the largest orange region
                x, y, w, h = cv2.boundingRect(largest_contour)
                self.get_logger().info(f'Center of orange region: ({cx}, {cy}), Width: {w}')
            else:
                self.get_logger().info('No valid orange region found')
        else:
            self.get_logger().info('No orange pixels detected')

def main(args=None):
    rclpy.init(args=args)
    attack_node = AttackNode()
    rclpy.spin(attack_node)
    attack_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
