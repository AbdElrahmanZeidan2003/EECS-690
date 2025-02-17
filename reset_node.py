import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class ResetNode(Node):
    def __init__(self):
        super().__init__('reset_node')
        # Subscriptions
        self.attack_sub = self.create_subscription(
            Empty, '/attack_complete', self.attack_complete_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher for robot motion commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Variables to hold position data
        self.initial_pose = None  # To store the initial position of the robot
        self.current_pose = None  # To store the current position of the robot continuously updated
        
        # State control
        self.position_capture_enabled = False  # Controls when to update the initial position

        self.get_logger().info('Reset node is initialized and waiting for signals.')

    def attack_complete_callback(self, msg):
        # Triggered when the attack is confirmed complete
        self.get_logger().info('Attack complete received. Ready to capture initial position post-spin.')
        self.position_capture_enabled = True  # Enable capturing of new initial position

    def odom_callback(self, msg):
        # Continuously updates current position and potentially updates initial position
        self.current_pose = msg.pose.pose
<<<<<<< HEAD
        
        
        self.move_to_initial()
    
    def get_yaw(self, orientation_q):
        """Convert quaternion to yaw (Euler angle)."""
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw
  
=======
        if self.position_capture_enabled:
            self.initial_pose = msg.pose.pose
            self.get_logger().info('Initial pose updated post-attack.')
            self.position_capture_enabled = False  # Reset to avoid continual updates

    def move_to_initial(self):
        # Calculate the distance and angle to the initial position and move towards it
        dx = self.initial_pose.position.x - self.current_pose.position.x
        dy = self.initial_pose.position.y - self.current_pose.position.y
        position_error = np.sqrt(dx**2 + dy**2)

        if position_error < 0.1:  # If close enough, stop moving
            self.stop_moving()
            self.get_logger().info('Reset complete! Robot is at initial pose.')
        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2 * position_error  # Simple proportional control
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info('Moving towards initial position.')

    def stop_moving(self):
        # Stops the robot by sending zero velocities
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('Movement stopped.')

def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetNode()
    rclpy.spin(reset_node)
    reset_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
>>>>>>> 2980c4f6826491f3ac35f8b2872cbdf628a97459
