import rclpy  
from rclpy.node import Node 
from geometry_msgs.msg import Twist  
from sensor_msgs.msg import Image, LaserScan
import time
import cv2
import numpy as np 
from cv_bridge import CvBridge


class SpinNode(Node):

    def __init__(self):
        super().__init__('spin_search')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rgb_subscription_ = self.create_subscription(Image, "/ascamera/camera_publisher/rgb0/image", self.published_image, 10)

        self.bridge = CvBridge()
        self.target_detected = False
        self.spin_complete = False
        self.red_cup_count = 0
  
        self.start_time = time.time()
        self.spin_duration = 25  
        self.spin_once()


    def spin_once(self):
        twist = Twist()
        twist.linear.x = 0.0 
        twist.angular.z = 0.5 

        # Publish the twist command
        self.get_logger().info('Spinning to detect cups...')
        end_time = time.time() + self.spin_duration
        while time.time() < end_time and rclpy.ok():
            self.publisher_.publish(twist)
            time.sleep(0.1)

        self.stop_spin()



    def stop_spin(self): 
        stop_twist = Twist()
        stop_twist.angular.z = 0.0
        self.publisher_.publish(stop_twist)  

        self.spin_complete = True
        self.get_logger().info(f"Spin complete. Red Cups Scanned: {self.red_cup_count}")

    def published_image(self, msg):
        if self.spin_complete:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h_mean = hsv[:, :, 0].mean()
        s_mean = hsv[:, :, 1].mean()
        v_mean = hsv[:, :, 2].mean()
        self.get_logger().info(f"HSV mean values - H: {h_mean:.2f}, S: {s_mean:.2f}, V: {v_mean:.2f} ")
        

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red2)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 + mask2
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        detected = len(contours) > 0

        if detected: 
            self.red_cup_count += 1
            self.get_logger().info(f"Red Cup Detected, Total Seen: {self.red_cup_count}")
        cv2.imshow("Camera Feed", frame)
        cv2.imshow("Red Masks", mask)
        cv2.waitKey(1)
        
        


def main(args=None):
    rclpy.init(args=args)
    spin_node = SpinNode()

    try:
        rclpy.spin(spin_node) 
    except KeyboardInterrupt:
        spin_node.get_logger().info("Spin node shutting down...")
    finally:
        if rclpy.ok():
            spin_node.destroy_node()  
            rclpy.shutdown()

if __name__ == '__main__':
    main()



