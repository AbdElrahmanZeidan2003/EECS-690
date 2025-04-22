import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import tf2_ros
import cv2
import numpy as np
import yaml
import subprocess
import math
import os
import random

class MazeRunner(Node):
    def __init__(self):
        super().__init__('maze_runner')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_scan = None
        self.goal_pose = None
        self.goal_saved = False
        self.reached = False
        self.phase = 'exploring'  # phases: exploring → saving_map → navigating → done
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
        if blue_area > 500 and not self.goal_saved:
            pose = self.get_current_pose()
            if pose:
                self.goal_pose = pose
                self.save_goal_pose(pose)
                self.goal_saved = True
                self.get_logger().info("Blue detected — saved pose. Switching to map save phase.")
                self.phase = 'saving_map'

    def main_loop(self):
        if self.phase == 'exploring':
            self.explore_step()
        elif self.phase == 'saving_map':
            self.save_map()
            self.phase = 'navigating'
        elif self.phase == 'navigating':
            self.drive_to_goal()
        elif self.phase == 'done':
            pass

    def explore_step(self):
        twist = Twist()
        if self.latest_scan:
            ranges = self.latest_scan.ranges
            num = len(ranges)
            front = min(ranges[0:10] + ranges[-10:])
            right = min(ranges[num//4-10:num//4+10])
            fr = min(ranges[num//8-5:num//8+5])

            if front < 0.25:
                twist.angular.z = 0.5
            elif right > 0.35:
                twist.angular.z = -0.3
                twist.linear.x = 0.05
            elif fr < 0.25:
                twist.angular.z = 0.3
                twist.linear.x = 0.05
            else:
                twist.linear.x = 0.15
        else:
            twist.linear.x = 0.05
        self.cmd_pub.publish(twist)

    def drive_to_goal(self):
        if not self.goal_pose or self.reached:
            return
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
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.reached = True
                self.phase = 'done'
                self.get_logger().info("Goal reached.")
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

    def save_goal_pose(self, pose):
        goal_dict = {
            'position': {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y
            },
            'orientation': {
                'z': pose.pose.orientation.z,
                'w': pose.pose.orientation.w
            }
        }
        path = '/home/YOUR_USERNAME/ros2_ws/src/class_pkg/config/slam_config.yaml'
        with open(path, 'w') as f:
            yaml.dump(goal_dict, f)

    def save_map(self):
        self.get_logger().info("Saving map using map_saver_cli...")
        map_path = '/home/YOUR_USERNAME/ros2_ws/src/class_pkg/config/map'
        try:
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path], check=True)
            self.get_logger().info(" Map saved.")
        except Exception as e:
            self.get_logger().error(f"Map saving failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MazeRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
