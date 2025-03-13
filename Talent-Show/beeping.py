immport rcplay
import time
from rcply.py import Node
from time  import sleep

class BeepNpde(Node):
    def __init__(self):
        super().__init__('beep_node')
        self.timer = self.creat_timer(2.0 , self.beep)
        
    def beep(self):
        self.get_logger().info("Beeping...")
def main(args=None):
    rclpy.init(args=args)
    node = BeepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting off...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()  
