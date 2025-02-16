'''
RILEY ANDERSON
02/09/2025
'''

from math import acos, cos, sqrt, degrees, pi
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
class Planner(Node):
    def __init__(self):
        # Initialize node with name 'Listener'
        super().__init__('Planner')
        # create subscription to 'coords' with msg type Float32MultiArray
        self.subscription = self.create_subscription(
        Float32MultiArray,
        'coords',
        self.listener_callback,
        10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.coords = []
    def listener_callback(self, msg):
        self.get_logger().info(f'Coordinates recieved: "{msg.data}"')
        self.coords = msg.data
        drive_commands = self.make_drive_commands(self.coords)
        for i in drive_commands:
            drive(i[0], i[1])

#this is a placeholder for the message from the topic
#coordinates will be in list form, with a tuple that is [angle (deg), distance]
# coords = [[15, 90], [47, 30], [170, 38], [230, 60], [300, 23]]

#make a new list with tuples [turn, drive]
    
    def make_drive_commands(coords):
        drive_commands = []
        for i in coords:
            #first tuple is command
            if i[0] == coords[0][0]:
                drive_commands.append(i)
                prev_ang = i[0]
                prev_dist = i[1]
            #soh cah toa!
            else:
                #law of cosines
                distance = (sqrt(i[1]**2 + prev_dist**2 - 2*i[1]*prev_dist*cos(prev_ang-i[0])))
                ang = degrees(acos((prev_dist**2 + i[1]**2 - distance**2)/(2*(prev_dist)*(i[1]))))
                print(f'[{distance}, {ang}]')
                drive_commands.append([ang, distance])
        return drive_commands
    """
    try this if the other doesn't work:
    
    def make_drive_commands(self, coords):  # Add self
    drive_commands = []
    for i in coords:
        if i[0] == coords[0][0]:
            drive_commands.append(i)
            prev_ang = i[0]
            prev_dist = i[1]
        else:
            # Law of Cosines correction
            distance = sqrt(i[1]**2 + prev_dist**2 - 2 * i[1] * prev_dist * cos(np.radians(prev_ang - i[0])))
            ang = degrees(acos((prev_dist**2 + i[1]**2 - distance**2) / (2 * prev_dist * i[1])))
            drive_commands.append([ang, distance])
    return drive_commands
    """
    #turn, then go
    def drive(degrees, distance):
        stop_msg = Twist()
        turn_msg = Twist()
        turn_msg.angular.x = 0.2
        go_msg = Twist()
        go_msg.linear.x = 0.2
        self.publisher.publish(turn_msg)
        time.sleep(turn_msg.angular.x / (degrees*10.6*(pi/180)))
        self.publisher.publish(go_msg)
        time.sleep(go_msg.linear.x / distance)
        self.publisher.publish(stop_msg)



def main(args=None):
    rclpy.init(args=args)
    listener_node = Planner()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
