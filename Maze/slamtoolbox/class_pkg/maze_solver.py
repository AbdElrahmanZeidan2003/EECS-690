import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import yaml
import os

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        # Load goal pose
        with open('/home/ubuntu/ros2_ws/src/class_pkg/config/slam_config.yaml', 'r') as file:
            goal_data = yaml.safe_load(file)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = goal_data['position']['x']
        goal_pose.pose.position.y = goal_data['position']['y']
        goal_pose.pose.orientation.z = goal_data['orientation']['z']
        goal_pose.pose.orientation.w = goal_data['orientation']['w']
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Navigating to saved goal...')
        self.navigator.goToPose(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
