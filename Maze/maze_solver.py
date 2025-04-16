import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import math
from cv_bridge import CvBridge
import cv2

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')
        # ROS interfaces
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.navigate_maze)
        self.bridge = CvBridge()
        # Maze grid
        self.grid_size = 5
        self.maze = np.full((self.grid_size, self.grid_size), 255)  # 255 = unexplored
        self.cost_map = np.full((self.grid_size, self.grid_size), 999)
        self.visited = np.zeros((self.grid_size, self.grid_size), dtype=bool)
        self.x, self.y = 0, 0  # start position
        self.heading = 0  # 0=N, 1=E, 2=S, 3=W
        # Goal detection
        self.goal_detected = False
        self.goal_cells = []

    def lidar_callback(self, msg):
        # Check for walls in front/left/right
        front = msg.ranges[0] < 0.35
        left = msg.ranges[90] < 0.35
        right = msg.ranges[-90] < 0.35
        # Update current cell with wall presence (simplified)
        if front:
            self.mark_wall(self.x, self.y, self.heading)
        if left:
            self.mark_wall(self.x, self.y, (self.heading - 1) % 4)
        if right:
            self.mark_wall(self.x, self.y, (self.heading + 1) % 4)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Detect blue color
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_area = cv2.countNonZero(mask)
        if blue_area > 500:  # threshold to confirm detection
            self.goal_detected = True
            self.goal_cells = [(self.x, self.y)]
            self.get_logger().info(f"Blue goal detected at ({self.x}, {self.y})")

    def mark_wall(self, x, y, direction):
        if self.maze[x][y] == 255:
            self.maze[x][y] = 0  # wall exists (simplified notation)

    def flood_fill(self):
        # BFS from goal
        queue = list(self.goal_cells)
        for gx, gy in self.goal_cells:
            self.cost_map[gx][gy] = 0
        while queue:
            cx, cy = queue.pop(0)
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = cx+dx, cy+dy
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    if self.maze[nx][ny] != 0 and self.cost_map[nx][ny] > self.cost_map[cx][cy] + 1:
                        self.cost_map[nx][ny] = self.cost_map[cx][cy] + 1
                        queue.append((nx, ny))

    def get_next_direction(self):
        min_cost = 999
        best_dir = None
        for i, (dx, dy) in enumerate([(0,1), (1,0), (0,-1), (-1,0)]):
            nx, ny = self.x+dx, self.y+dy
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                if self.maze[nx][ny] != 0 and self.cost_map[nx][ny] < min_cost:
                    min_cost = self.cost_map[nx][ny]
                    best_dir = i
        return best_dir

    def rotate_to(self, desired_heading):
        turn_cmd = Twist()
        if (desired_heading - self.heading) % 4 == 1:
            turn_cmd.angular.z = -0.5  # right
        elif (self.heading - desired_heading) % 4 == 1:
            turn_cmd.angular.z = 0.5  # left
        else:
            return
        self.cmd_pub.publish(turn_cmd)
        self.heading = desired_heading

    def navigate_maze(self):
        if self.goal_detected and (self.x, self.y) in self.goal_cells:
            self.get_logger().info('Goal reached at blue paper!')
            return
        if not self.goal_detected:
            self.get_logger().info('Searching for blue paper...')
        self.flood_fill()
        direction = self.get_next_direction()
        if direction is None:
            self.get_logger().info('Stuck. No direction.')
            return
        self.rotate_to(direction)
        # Move forward
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        self.cmd_pub.publish(move_cmd)

        # Update pose
        dx, dy = [(0,1), (1,0), (0,-1), (-1,0)][direction]
        self.x += dx
        self.y += dy
        self.visited[self.x][self.y] = True

def main(args=None):
    rclpy.init(args=args)
    node = MazeSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
