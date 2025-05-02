import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge


class Vision(Node):
    BALL_DIAM = 0.038
    FOCAL_PX = 554.0
    FAR_DIST = 0.80
    RADIUS_FAR = (FOCAL_PX * BALL_DIAM) / (2 * FAR_DIST)

    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.attack = False
        self.defense = False

        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)

        self.pub_attack = self.create_publisher(Bool, '/robot/attack', 10)
        self.pub_defend = self.create_publisher(Bool, '/robot/defense', 10)
        self.pub_dist = self.create_publisher(Float32, '/robot/ball_distance', 10)
        self.pub_angle = self.create_publisher(Float32, '/robot/ball_angle', 10)

        
        self.lower_red = np.array([0, 0, 140], dtype=np.uint8)
        self.upper_red = np.array([80, 80, 255], dtype=np.uint8)

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        found, (cx, cy, r_px) = self.detect_ball(frame)
        if not found:
            return

        dist_m = (self.FOCAL_PX * self.BALL_DIAM) / (2 * r_px)
        angle_r = math.atan2(cx - frame.shape[1] / 2, self.FOCAL_PX)

        self.pub_dist.publish(Float32(data=dist_m))
        self.pub_angle.publish(Float32(data=angle_r))

        self.attack = r_px >= self.RADIUS_FAR
        self.defense = not self.attack
        self.pub_attack.publish(Bool(data=self.attack))
        self.pub_defend.publish(Bool(data=self.defense))

    def detect_ball(self, img):
        mask = cv2.inRange(img, self.lower_red, self.upper_red)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return False, (0, 0, 0)

        c = max(cnts, key=cv2.contourArea)
        (x, y), r = cv2.minEnclosingCircle(c)
        if r < 3:
            return False, (0, 0, 0)
        return True, (x, y, r)


def main(args=None):
    rclpy.init(args=args)
    node = Vision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
