#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import random
import heapq

class AStar:
    def __init__(self, grid):
        self.grid = grid

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        return path

    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            current_cost, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, goal)

            for next in self.grid.neighbors(current):
                tentative_g_score = g_score[current] + 1
                if next not in g_score or tentative_g_score < g_score[next]:
                    came_from[next] = current
                    g_score[next] = tentative_g_score
                    f_score = tentative_g_score + self.heuristic(next, goal)
                    heapq.heappush(open_set, (f_score, next))

class Grid:
    def __init__(self, width, height):
        self.width = width
        self.height = height

    def neighbors(self, pos):
        x, y = pos
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                if 0 <= x + i < self.width and 0 <= y + j < self.height:
                    yield (x + i, y + j)

class TurtleGame(Node):
    def __init__(self):
        super().__init__('turtle_game')
        self.declare_parameter('thief_linear_speed', 1.0)
        self.declare_parameter('thief_angular_speed', 2.0)  # Higher angular speed for more randomness
        self.declare_parameter('cop_linear_speed', 1.5)  # Higher linear speed for faster catching
        self.declare_parameter('cop_angular_speed', 1.0)

        self.thief_linear_speed = self.get_parameter('thief_linear_speed').get_parameter_value().double_value
        self.thief_angular_speed = self.get_parameter('thief_angular_speed').get_parameter_value().double_value
        self.cop_linear_speed = self.get_parameter('cop_linear_speed').get_parameter_value().double_value
        self.cop_angular_speed = self.get_parameter('cop_angular_speed').get_parameter_value().double_value

        self.cop_pose = None
        self.thief_pose = None
        
        self.pub_cop = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_thief = self.create_publisher(Twist, '/thief/cmd_vel', 10)
        
        self.sub_cop = self.create_subscription(Pose, '/turtle1/pose', self.cop_pose_callback, 10)
        self.sub_thief = self.create_subscription(Pose, '/thief/pose', self.thief_pose_callback, 10)
        
        self.create_timer(0.1, self.update)  # Update at 10 Hz

        # Spawn thief turtle
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Spawn.Request()
        self.req.x = random.uniform(1.0, 10.0)  # Random x within the screen boundaries (1 to 10)
        self.req.y = random.uniform(1.0, 10.0)  # Random y within the screen boundaries (1 to 10)
        self.req.theta = 0.0
        self.req.name = 'thief'
        self.future = self.cli.call_async(self.req)

        # Spawn cop turtle
        self.req.x = random.uniform(1.0, 10.0)  # Random x within the screen boundaries (1 to 10)
        self.req.y = random.uniform(1.0, 10.0)  # Random y within the screen boundaries (1 to 10)
        self.req.name = 'turtle1'  # Name of the default turtle
        self.future = self.cli.call_async(self.req)

        self.screen_width = 11.0
        self.screen_height = 11.0

        self.grid = Grid(int(self.screen_width), int(self.screen_height))

    def cop_pose_callback(self, msg):
        self.cop_pose = msg

    def thief_pose_callback(self, msg):
        self.thief_pose = msg

    def update(self):
        if self.cop_pose and self.thief_pose:
            # Check if the cop is close enough to catch the thief
            distance_to_thief = math.sqrt((self.thief_pose.x - self.cop_pose.x)**2 + (self.thief_pose.y - self.cop_pose.y)**2)
            if distance_to_thief < 0.5:  # Adjust this threshold as needed
                # Thief caught, print message
                self.get_logger().info("Thief was caught at coordinates: ({}, {})".format(self.thief_pose.x, self.thief_pose.y))
                return  # Exit update function

            # Integrate A* algorithm to compute optimal path for the thief
            start = (int(self.thief_pose.x), int(self.thief_pose.y))
            goal = self.compute_thief_goal()
            path = AStar(self.grid).astar(start, goal)
            if path:
                # Move thief along the computed path
                next_position = path[0]
                angle_to_next = math.atan2(next_position[1] - self.thief_pose.y, next_position[0] - self.thief_pose.x)
                distance_to_next = math.sqrt((next_position[0] - self.thief_pose.x)**2 + (next_position[1] - self.thief_pose.y)**2)

                # Move the thief towards the next position
                thief_msg = Twist()
                thief_msg.linear.x = min(distance_to_next, self.thief_linear_speed)
                thief_msg.angular.z = angle_to_next - self.thief_pose.theta
                self.pub_thief.publish(thief_msg)

            angle_to_thief = math.atan2(self.thief_pose.y - self.cop_pose.y,
                                        self.thief_pose.x - self.cop_pose.x)
            
            # Check if the cop is about to collide with the border
            if self.is_about_to_collide(self.cop_pose):
                # Change direction if about to collide
                self.cop_linear_speed *= -1

            # Calculate the angle difference between the cop's orientation and angle to the thief
            angle_difference = abs(angle_to_thief - self.cop_pose.theta)

            if angle_difference > 0.1:  # Adjust this threshold as needed
                # Rotate in place to face towards the thief
                cop_msg = Twist()
                cop_msg.angular.z = self.cop_angular_speed if angle_to_thief > self.cop_pose.theta else -self.cop_angular_speed
                self.pub_cop.publish(cop_msg)
            else:
                # Move forward in the direction of the thief
                cop_msg = Twist()
                cop_msg.linear.x = self.cop_linear_speed
                cop_msg.linear.y = 0.0
                cop_msg.angular.x =0.0
                cop_msg.angular.y = 0.0
                cop_msg.angular.z = self.cop_angular_speed * (angle_to_thief - self.cop_pose.theta)
                self.pub_cop.publish(cop_msg)

    def compute_thief_goal(self):
        # Calculate a goal position that maximizes the distance from the cop
        # For simplicity, we can use a point on the opposite side of the grid from the cop
        # You can customize this logic further based on your requirements
        goal_x = self.screen_width - self.cop_pose.x
        goal_y = self.screen_height - self.cop_pose.y
        return (int(goal_x), int(goal_y))

    def is_about_to_collide(self, pose):
        if (pose.x <= 1.0 or pose.x >= self.screen_width - 1.0 or
            pose.y <= 1.0 or pose.y >= self.screen_height - 1.0):
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    turtle_game = TurtleGame()
    rclpy.spin(turtle_game)
    turtle_game.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

