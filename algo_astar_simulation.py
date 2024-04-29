#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import random
from pathfinding.core.grid import Grid
from pathfinding.core.node import Node as PFNode
from pathfinding.finder.a_star import AStarFinder

class TurtleGame(Node):
    def __init__(self):
        super().__init__('turtle_game')
        self.declare_parameter('thief_linear_speed', 1.0)
        self.declare_parameter('thief_angular_speed', 2.0)
        self.declare_parameter('cop_linear_speed', 1.5)
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
        self.req.x = random.uniform(1.0, 10.0)
        self.req.y = random.uniform(1.0, 10.0)
        self.req.theta = 0.0
        self.req.name = 'thief'
        self.future = self.cli.call_async(self.req)

        # Spawn cop turtle
        self.req.x = random.uniform(1.0, 10.0)
        self.req.y = random.uniform(1.0, 10.0)
        self.req.name = 'turtle1'
        self.future = self.cli.call_async(self.req)

        self.screen_width = 11.0
        self.screen_height = 11.0

    def cop_pose_callback(self, msg):
        self.cop_pose = msg

    def thief_pose_callback(self, msg):
        self.thief_pose = msg

    def update(self):
        if self.cop_pose and self.thief_pose:
            # Define the grid representation of the environment
            grid = Grid(width=int(self.screen_width), height=int(self.screen_height))

            # Update the grid with obstacles (edges of the screen)
            for x in range(int(self.screen_width)):
                grid.node(x, 0).walkable = False
                grid.node(x, int(self.screen_height) - 1).walkable = False
            for y in range(int(self.screen_height)):
                grid.node(0, y).walkable = False
                grid.node(int(self.screen_width) - 1, y).walkable = False

            # Convert the cop and thief positions to grid coordinates
            cop_x, cop_y = int(self.cop_pose.x), int(self.cop_pose.y)
            thief_x, thief_y = int(self.thief_pose.x), int(self.thief_pose.y)

            # Find the path from the thief to a safe location using A* algorithm
            start_node = (cop_x, cop_y)
            end_node = (thief_x, thief_y)

            finder = AStarFinder()
            path, _ = finder.find_path(start_node, end_node, grid)

            if len(path) > 0:
                # Calculate the next move towards the safe location
                next_x, next_y = path[1]  # Index 0 is the current position, index 1 is the next position

                # Calculate the angle towards the next position
                angle_to_next = math.atan2(next_y - thief_y, next_x - thief_x)

                # Move forward in the direction of the next position
                thief_msg = Twist()
                thief_msg.linear.x = self.thief_linear_speed
                thief_msg.angular.z = self.thief_angular_speed * (angle_to_next - self.thief_pose.theta)
                self.pub_thief.publish(thief_msg)

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

