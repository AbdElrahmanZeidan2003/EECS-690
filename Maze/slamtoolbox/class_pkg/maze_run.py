import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
import random

class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.bridge = CvBridge()
        self.navigator = BasicNavigator()
        #self.navigator.waitUntilNav2Active()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.explore_step)
        self.latest_scan = None
        self.goal_sent = False  # Only navigate once

    def lidar_callback(self, msg):
        self.latest_scan = msg
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        # Need to double check this 
        lower_blue = np.array([0, 0, 0])
        upper_blue = np.array([255, 255, 104])
        mask = cv2.inRange(lab, lower_blue, upper_blue)
        blue_area = cv2.countNonZero(mask)
        if blue_area > 500 and not self.goal_sent:
            self.get_logger().info('Blue paper detected (LAB) — navigating to current location...')
            goal_pose = self.get_current_pose()
            if goal_pose:
                self.navigator.goToPose(goal_pose)
                self.goal_sent = True

    def explore_step(self):
    if self.goal_sent:
        return  # Stop exploring once navigating to goal

    if not self.latest_scan:
        # No LiDAR data yet — creep forward slowly
        twist = Twist()
        twist.linear.x = 0.05
        self.cmd_pub.publish(twist)
        return

    ranges = self.latest_scan.ranges
    n = len(ranges)

    # Define LiDAR sectors
    front = min(ranges[0:10] + ranges[-10:])
    right = min(ranges[n//4 - 10 : n//4 + 10])        # 90°
    front_right = min(ranges[n//8 - 10 : n//8 + 10])  # 45°
    wall_distance = 0.35
    stop_threshold = 0.25
    twist = Twist()
    if front < stop_threshold:
        # Wall in front — turn left
        twist.angular.z = 0.5
        twist.linear.x = 0.0
    elif right > wall_distance:
        # No wall on right — turn right to find it
        twist.angular.z = -0.3
        twist.linear.x = 0.05
    elif front_right < stop_threshold:
        # Wall diagonally ahead — steer away
        twist.angular.z = 0.3
        twist.linear.x = 0.05
    else:
        # Clear path — go forward
        twist.linear.x = 0.15
        twist.angular.z = 0.0
    self.cmd_pub.publish(twist)


    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

