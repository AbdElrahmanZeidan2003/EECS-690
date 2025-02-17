import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import numpy as np

class ResetNode(Node):
    def __init__(self):
        super().__init__('reset_node')
       
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        

        self.initial_pose = None
        self.current_pose = None
        
  
        self.kp_linear = 0.5 
        self.kp_angular = 1.0  
        
       
        self.position_threshold = 0.1 
        self.orientation_threshold = 0.2  

    def odom_callback(self, msg):
        
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.get_logger().info("Initial pose saved!")
        
    
        self.current_pose = msg.pose.pose
        
        
        self.move_to_initial()
    
    def get_yaw(self, orientation_q):
        """Convert quaternion to yaw (Euler angle)."""
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw
  