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
        if self.latest_scan:
            ranges = [(i, r) for i, r in enumerate(self.latest_scan.ranges) if r > 0.05 and r < self.latest_scan.range_max]
            num_scans = len(ranges)
            if  10.0 >= min(ranges[num_scans//4-5:num_scans//4+5])[1]:
                twist = Twist()
                twist.linear.x = 0.1
                twist.linear.y = 0.0
                self.cmd_pub.publish(twist)
            elif min(ranges[num_scans//4-5:num_scans//4+5])[1] <= 10.0 and min(ranges[:10])[1] >= 5.0:
                twist = Twist()
                twist.linear.x = 0.1
                twist.linear.y = 0.1
                self.cmd_pub.publish(twist)
            else:
                twist = Twist()
                twist.linear.x = 0.1
                twist.linear.y = -0.1
                self.cmd_pub.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = 0.1  # move forward slowly if no LiDAR yet
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

