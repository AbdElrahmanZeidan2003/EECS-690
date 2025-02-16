import rclpy  
from rclpy.node import Node 
from geometry_msgs.msg import Twist  
import time  

class SpinNode(Node):

    def __init__(self):
        super().__init__('spin_search')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.running = True
        self.timer = self.create_timer(0.5, self.spin)  
        self.start_time = time.time()
        self.spin_duration = 5  

    def spin(self):
        if not self.running:
            return 
        if time.time() - self.start_time > self.spin_duration:
            self.stop_spin()
            return
        twist = Twist()
        twist.linear.x = 0.0 
        twist.angular.z = 0.5 

        # Publish the twist command
        self.publisher_.publish(twist)
        self.get_logger().info('Spinning to detect cups...')

    def stop_spin(self):
        self.running = False 
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)  

        self.get_logger().info("Spin complete. Stopping rotation.")

def main(args=None):
    rclpy.init(args=args)
    spin_node = SpinNode()
    try:
        rclpy.spin(spin_node) 
    except KeyboardInterrupt:
        spin_node.get_logger().info("Spin node shutting down...")
        spin_node.stop_spin()
    finally:
        if rclpy.ok():
            spin_node.destroy_node()  
            rclpy.shutdown()

if __name__ == '__main__':
    main()
