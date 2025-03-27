import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class Orange(Node):
    def __init__(self):
        super().__init__('orange')
        self.safety = False
        self.subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            1)
        self.bridge = CvBridge()

        self.safety = self.create_subscription(
            Bool,
            '/safety',
            self.safety_callback,
            1)

        self.drive_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1)
        
        self.angle_pub = self.create_publisher(
            Float32,
            '/ang',
            1)
        
    def safety_callback(self, msg):
        if msg.data == True:
            self.safety = True
        else:
            self.safety = False

    def image_callback(self, msg):
        try:
            # ROS2 image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return
        
        # convert BGR image to HSV
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
                #cy = int(M['m01'] / M['m00'])
                
                # bounding box of the largest orange region
                x, y, w, h = cv2.boundingRect(largest_contour)
                self.get_logger().info(f'Center of orange region: ({cx}), Width: {w}')
            else:
                self.get_logger().info('No valid orange region found')
        else:
            self.get_logger().info('No orange pixels detected')
        
        # for debugging
        # cv2.imshow("Orange Mask", mask)
        # cv2.waitKey(1)


        def drive_to(self, x):
            #SET HORIZONTAL RESOLUTION
            resolution = 1080
            #acceptable portion of frame to be considered 'center'
            center_range = (1/5) * resolution
            #distance from center of frame
            dcenter = x-(resolution/2)
            #maximum speed of turn
            max_angular = 4.0

            if abs(dcenter)<=center_range:
                #avoid div by zero
                if dcenter == 0:
                    rotate_speed = 0
                #point is in center range
                else:
                    rotate_speed = (dcenter/8)*max_angular

            else:
                rotate_speed = max_angular*(dcenter/4)
            
            return rotate_speed

                


        def publish_twist_message(self, forward, horizontal, angular):
            msg = Twist
            msg.linear.x = horizontal
            msg.linear.y = forward
            msg.angular.z = angular
            


def main(args=None):
    rclpy.init(args=args)
    node = Orange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
