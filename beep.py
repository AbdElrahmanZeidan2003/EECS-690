import rclpy
import time 
from rclpy.node import Node 
from time import sleep

class BeepNode(Node):
    def __init__(self):
        super().__init__('beep_node')
        self.timer = self.creat_timer(2.0, self.beep)
    def beep(self):
        self.get_logger().into("beeping...")
def main(args=None):
    rclpy.init(args=args)
    beep_node = BeepNode()
    try:
        rclpy.spin(beep_node)
    except KeyboardInterrupt:
        beep_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
