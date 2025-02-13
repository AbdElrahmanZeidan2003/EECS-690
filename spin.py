import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SpinNode(Node):
    def __init__(self):
        super().__init__('spin_search')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.running = True
        self.timer = self.create_timer(0.5, self.spin)

    def spin(self):
        if not self.running:
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0 
        twist.angular.z = 0.0

        self.publisher_.publish(twist)
        self.get_logger().info('Stop spin...')

def main(args=None):
    rclpy.init(args=args)
    spin_node = SpinNode()

    try:
        rclpy.spin(spin_node)
    except KeyboardInterrupt:
        spin_node.get_logger().info("Spin node shutting down...")
        spin_node.stop()
    finally:
        if rclpy.ok():
            spin_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
