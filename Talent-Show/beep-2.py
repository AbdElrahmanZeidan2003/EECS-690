import rclpy  
from rclpy.node import Node  
from std_msgs.msg import String  
import time  

class BeepNode(Node):

    def __init__(self):
        super().__init__('beep_node')

        # Subscriber for beep events (HIT CONFIRM, ERROR)
        self.subscription = self.create_subscription(
            String, '/beep_signal', self.beep_callback, 10)

        # Subscriber for spin status (SPIN START, SPIN STOP)
        self.spin_subscription = self.create_subscription(
            String, '/spin_status', self.spin_callback, 10)

        self.spinning = False  

    def beep_callback(self, msg):

        # Handles beeping events for attack confirmation and errors.
    
        if msg.data == "HIT CONFIRM":
            self.get_logger().info("Beeping: Hit confirmed!")
            self.simulate_beep(2)  

        elif msg.data == "ERROR: BLUE CUP HIT":
            self.get_logger().warning("Beeping: Blue cup was hit!")
            self.simulate_beep(3) 

    def spin_callback(self, msg):
        # Handles beeping during spinning. If spinning starts, beep continuously every second.
        if msg.data == "SPIN START":
            self.get_logger().info("Beeping: Robot is spinning to detect cups!")
            self.spinning = True
            while self.spinning and rclpy.ok():
                self.simulate_beep(1)  
                time.sleep(1)  

        elif msg.data == "SPIN STOP":
            self.get_logger().info("Stopping beep: Spin complete.")
            self.spinning = False  

    def simulate_beep(self, times=1):
        """
        Simulates a beep by logging to the console (replace with real buzzer code).
        """
        for _ in range(times):
            self.get_logger().info("BEEP!")  
            time.sleep(0.2)  

def main(args=None):
    rclpy.init(args=args)
    beep_node = BeepNode()
    rclpy.spin(beep_node)
    beep_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
