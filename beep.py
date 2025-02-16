import rclpy  
from rclpy.node import Node 
from std_msgs.msg import String  
import time  

class BeepNode(Node):
    def __init__(self):
        super().__init__('beep_node')
        self.subscription = self.create_subscription(
            String, '/beep_signal', self.beep_callback, 10)

        self.get_logger().info("Beep node initialized. Waiting for beep signals...")

    def beep_callback(self, msg):
        if msg.data == "HIT CONFIRM":
            self.get_logger().info("Beeping: Hit confirmed!")  
            self.simulate_beep(2)  
        
        elif msg.data == "ERROR: BLUE CUP HIT":
            self.get_logger().warning("Beeping: Blue cup was hit! Error!")  
            self.simulate_beep(3)  
        
        elif msg.data == "RESET_STAGE":
            self.get_logger().info("Beeping: Reset mode triggered!")  
            self.simulate_beep(1) 
        
    def simulate_beep(self, times=1):
        for _ in range(times):
            self.get_logger().info("BEEP!")  
            time.sleep(0.2)  

def main(args=None):
    rclpy.init(args=args)
    beep_node = BeepNode()
    try:
        rclpy.spin(beep_node) 
    except KeyboardInterrupt:
        beep_node.get_logger().info("Beep node shutting down...")
    finally:
        if rclpy.ok():
            beep_node.destroy_node()  
            rclpy.shutdown()

if __name__ == '__main__':
    main()
