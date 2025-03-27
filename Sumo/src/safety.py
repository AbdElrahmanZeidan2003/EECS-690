import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class Safety(Node):
    def __init__(self):
        super().__init__('safety')
        self.safety_flag = False
        self.collision = False
        self.bridge = CvBridge()

        # Subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/rgb0/image',
            self.image_callback,
            1)
        self.safety_subscription = self.create_subscription(
            Bool,
            '/safety',
            self.safety_callback,
            1)
        self.depth_subscription = self.create_subscription(
            Image,
            '/ascamera/camera_publisher/depth0/image',
            self.check_collision,
            1)

        # Publishers
        self.drive_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.angle_pub = self.create_publisher(Float32, '/ang', 1)

    def safety_callback(self, msg):
        self.safety_flag = msg.data

    def image_callback(self, msg):
        try:
            # Convert ROS2 image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # Convert BGR image to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define orange color range
        lower_orange = np.array([0, 165, 104])
        upper_orange = np.array([255, 255, 255])

        # Threshold the image to get only orange pixels
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find contours of the orange regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour based on area
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])

                # Bounding box of the largest orange region
                x, y, w, h = cv2.boundingRect(largest_contour)
                self.get_logger().info(f'Center of orange region: ({cx}), Width: {w}')
                if self.collision:
                    self.publish_twist_message(-2.0, 0.0, self.calc_rotation(cx))
                else:
                    self.publish_twist_message(2.0, 0.0, self.calc_rotation(cx))
            else:
                self.get_logger().info('No valid orange region found')
                self.publish_twist_message(0, 0.3, 2)
        else:
            self.get_logger().info('No orange pixels detected')

    def calc_rotation(self, x):
        # Set horizontal resolution
        resolution = 1080
        # Acceptable portion of frame to be considered 'center'
        center_range = (1/5) * resolution
        # Distance from center of frame
        dcenter = x - (resolution / 2)
        # Maximum speed of turn
        max_angular = 4.0

        if abs(dcenter) <= center_range:
            # Avoid division by zero
            if dcenter == 0:
                rotate_speed = 0
            else:
                rotate_speed = (dcenter / 8) * max_angular
        else:
            rotate_speed = max_angular * (dcenter / 4)

        return rotate_speed

    def publish_twist_message(self, forward, horizontal, angular):
        if not self.safety_flag:
            msg = Twist()
            msg.linear.x = horizontal
            msg.linear.y = forward
            msg.angular.z = angular
            self.drive_pub.publish(msg)
        ang_msg = Float32()
        ang_msg.data = angular
        self.angle_pub.publish(ang_msg)

    def check_collision(self, msg):
        collision_threshold = 0.02
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Could not convert depth image: {e}")
            return

        height, width = depth_image.shape
        center_region = depth_image[height//2-1:height//2+2, width//2-1:width//2+2]
        avg_depth = np.nanmean(center_region)

        if avg_depth > collision_threshold:
            self.collision = False
        else:
            self.collision = True

def main(args=None):
    rclpy.init(args=args)
    node = Safety()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
