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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.explore_step)
        # State flags
        self.started = False  # red seen
        self.goal_sent = False  # blue goal sent
        self.seen_blue = False
        self.latest_scan = None

    def lidar_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Detect RED
        red_mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        red_area = cv2.countNonZero(red_mask)
        if red_area > 500 and not self.started:
            self.get_logger().info('Red paper detected — starting maze exploration')
            self.started = True
        # Detect BLUE
        blue_mask = cv2.inRange(hsv, np.array([100, 150, 50]), np.array([140, 255, 255]))
        blue_area = cv2.countNonZero(blue_mask)
        if blue_area > 500 and not self.goal_sent:
            self.get_logger().info('Blue paper detected! Getting position to send goal.')
            self.seen_blue = True

    def explore_step(self):
        if not self.started or self.goal_sent:
            return

        if self.seen_blue:
            goal_pose = self.get_current_pose()
            if goal_pose:
                self.navigator.goToPose(goal_pose)
                self.get_logger().info('Sent navigation goal to blue paper location!')
                self.goal_sent = True
            return
        # Simple explore logic: go forward if clear, turn if blocked
        twist = Twist()
        if self.latest_scan:
            front_ranges = self.latest_scan.ranges[0:10] + self.latest_scan.ranges[-10:]
            min_front = min(front_ranges)
            if min_front > 0.4:
                twist.linear.x = 0.15
            else:
                twist.angular.z = random.choice([-1.0, 1.0])  # turn randomly
                twist.linear.x = 0.0
        else:
            twist.linear.x = 0.1  # move gently if no scan yet
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
            self.get_logger().warn(f'TF error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
