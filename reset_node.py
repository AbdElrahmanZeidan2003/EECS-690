import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np

class ResetNode(Node):
    def __init__(self):
        super().__init__('reset_node')
        self.subscription = self.create_subscription(
            Empty, '/attack_complete', self.attack_complete_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.initial_pose = None
        self.current_pose = None
        self.get_logger().info('Reset node is ready and waiting for attack to complete.')

    def attack_complete_callback(self, msg):
        self.get_logger().info('Attack complete received. Starting reset process.')
        if self.initial_pose is None:  # Ensure we have an initial pose to move to
            self.get_logger().info('Initial pose not set, cannot reset.')
        else:
            self.move_to_initial()

    def odom_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.get_logger().info('Initial pose saved!')
        self.current_pose = msg.pose.pose

    def move_to_initial(self):
        dx = self.initial_pose.position.x - self.current_pose.position.x
        dy = self.initial_pose.position.y - self.current_pose.position.y
        position_error = np.sqrt(dx**2 + dy**2)

        if position_error < 0.1:  # If close to the initial pose, stop moving
            self.stop_moving()
            self.get_logger().info('Reset complete! Robot is at initial pose.')
        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2 * position_error
            self.publisher.publish(cmd_vel)
            self.get_logger().info('Moving towards initial position.')

    def stop_moving(self):
        cmd_vel = Twist()  # Stop the robot
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetNode()
    rclpy.spin(reset_node)
    reset_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
