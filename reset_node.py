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
    """
    try this if get_yaw doesn't work:
    
    def get_yaw(self, orientation_q):
    if orientation_q is None:
        return 0  # Default yaw if orientation is None
    _, _, yaw = euler_from_quaternion(
        [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    return yaw
    
    """
    def move_to_initial(self):
        if self.initial_pose is None or self.current_pose is None:
            return 
        
      
        dx = self.initial_pose.position.x - self.current_pose.position.x
        dy = self.initial_pose.position.y - self.current_pose.position.y
        position_error = np.sqrt(dx**2 + dy**2)
        
        target_yaw = self.get_yaw(self.initial_pose.orientation)
        current_yaw = self.get_yaw(self.current_pose.orientation)
        error_yaw = (target_yaw - current_yaw + np.pi) % (2 * np.pi) - np.pi  
        
       
        cmd_vel = Twist()
        

        cmd_vel.linear.x = self.kp_linear * position_error
        
        cmd_vel.angular.z = self.kp_angular * error_yaw
    
        cmd_vel.linear.x = max(min(cmd_vel.linear.x, 0.6), -0.6) 
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, 2.0), -2.0)  
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        if position_error < self.position_threshold and abs(error_yaw) < self.orientation_threshold:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info("Reset complete! Robot is at initial pose.")
            rclpy.shutdown()  

def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetNode()
    rclpy.spin(reset_node)
    reset_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



"""
try this if it doesn't work:


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
        if orientation_q is None:
            return 0  # Default yaw if orientation is None
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw

    def move_to_initial(self):
        if self.initial_pose is None or self.current_pose is None:
            return
        
        dx = self.initial_pose.position.x - self.current_pose.position.x
        dy = self.initial_pose.position.y - self.current_pose.position.y
        position_error = np.sqrt(dx**2 + dy**2)
        target_yaw = self.get_yaw(self.initial_pose.orientation)
        current_yaw = self.get_yaw(self.current_pose.orientation)
        error_yaw = (target_yaw - current_yaw + np.pi) % (2 * np.pi) - np.pi  
        cmd_vel = Twist()
        cmd_vel.linear.x = self.kp_linear * position_error
        cmd_vel.angular.z = self.kp_angular * error_yaw
        cmd_vel.linear.x = max(min(cmd_vel.linear.x, 0.6), -0.6)  
        cmd_vel.angular.z = max(min(cmd_vel.angular.z, 2.0), -2.0)  

        self.cmd_vel_pub.publish(cmd_vel)

        if position_error < self.position_threshold and abs(error_yaw) < self.orientation_threshold:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info("Reset complete! Robot is at initial pose.")
            rclpy.shutdown() 

def main(args=None):
    rclpy.init(args=args)
    reset_node = ResetNode()
    rclpy.spin(reset_node)
    reset_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""