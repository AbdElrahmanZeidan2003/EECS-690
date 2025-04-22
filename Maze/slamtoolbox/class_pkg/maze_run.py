import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
import random
import math
import yaml
import os

class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_scan = None
        self.goal_pose = None
        self.goal_reached = False
        self.blue_seen = False
        self.timer = self.create_timer(0.3, self.main_loop)

    def lidar_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        lower_blue = np.array([0, 0, 0])
        upper_blue = np.array([255, 255, 104])
        mask = cv2.inRange(lab, lower_blue, upper_blue)
        blue_area = cv2.countNonZero(mask)

        if blue_area > 500 and not self.blue_seen:
            pose = self.get_current_pose()
            if pose:
                self.goal_pose = pose
                self.blue_seen = True
                self.get_logger().info("Blue paper detected")

    def main_loop(self):
        if not self.blue_seen:
            self.explore_step()
        elif not self.goal_reached:
            self.drive_to_goal()

    def explore_step(self):
        twist = Twist()
        if not self.latest_scan:
            twist.linear.x = 0.05
            self.cmd_pub.publish(twist)
            return
        ranges = self.latest_scan.ranges
        n = len(ranges)
        front = min(ranges[0:10] + ranges[-10:])
        right = min(ranges[n//4 - 5 : n//4 + 5])      # 90°
        front_right = min(ranges[n//8 - 5 : n//8 + 5])  # 45°
        wall_distance = 0.35
        stop_threshold = 0.25
        if front < stop_threshold:
            twist.angular.z = 0.5
            twist.linear.x = 0.0
        elif right > wall_distance:
            twist.angular.z = -0.3
            twist.linear.x = 0.05
        elif front_right < stop_threshold:
            twist.angular.z = 0.3
            twist.linear.x = 0.05
        else:
            twist.linear.x = 0.15
            twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def drive_to_goal(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            dx = self.goal_pose.pose.position.x - x
            dy = self.goal_pose.pose.position.y - y
            distance = math.hypot(dx, dy)
            angle = math.atan2(dy, dx)
            yaw = self.get_yaw_from_quaternion(trans.transform.rotation)
            error = angle - yaw
            twist = Twist()
            if distance > 0.2:
                twist.linear.x = 0.1
                twist.angular.z = error
            else:
                self.goal_reached = True
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("🎯 Reached blue paper.")
            self.cmd_pub.publish(twist)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def get_yaw_from_quaternion(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

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
