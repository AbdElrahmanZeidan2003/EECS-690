import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class AttackNode(Node):
    def __init__(self):
        super().__init__('attack_node')
        self.attack_enabled = False
        
        #Publisher for sending velocity commands to the robot
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #Subscriber for receiving boolean from perception node
        #"/topic_name" is just placeholder in place of actual topic name
        self.create_subscription(Bool, '/topic_name', self.attack_callback, 10)
        
        #Timer to keep sending attack commands
        self.timer = self.create_timer(0.1, self.timer_callback)

        #Logger for debugging
        self.get_logger().info('Attack mode initialized. Waiting for attack trigger...')

    def attack_callback(self, msg: Bool):
        #Set attack to True or False based on msg.data to start or stop attack
        self.attack_enabled = msg.data
        if self.attack_enabled:
            self.get_logger().info('Attack trigger initiated: Beginning attack.')
        else:
            self.get_logger().info('Attack trigger deactivated: Stopping attack.')

    def timer_callback(self):
        twist = Twist()
        if self.attack_enabled:
            #Move forward to attack enemy
            twist.linear.x = 0.5
            twist.angular.z = 0.0
        else:
            #Stop command
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    attack_node = AttackNode()
    rclpy.spin(attack_node)
    attack_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
