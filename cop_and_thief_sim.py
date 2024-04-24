#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import random

class TurtleGame(Node):
    def __init__(self):
        super().__init__('turtle_game')
        self.declare_parameter('thief_speed', 1.0)
        self.declare_parameter('cop_speed', 0.5)

        self.thief_speed = self.get_parameter('thief_speed').get_parameter_value().double_value
        self.cop_speed = self.get_parameter('cop_speed').get_parameter_value().double_value

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

    def cop_pose_callback(self, msg):
        self.cop_pose = msg
        #if self.cop_pose:
         #   self.get_logger().info('Cop Pose: x=%.2f, y=%.2f' % (self.cop_pose.x, self.cop_pose.y))

    def thief_pose_callback(self, msg):
        self.thief_pose = msg
        #if self.thief_pose:
         #   self.get_logger().info('Thief Pose: x=%.2f, y=%.2f' %(self.thief_pose.x, self.thief_pose.y))

    def update(self):
        if self.cop_pose and self.thief_pose:
            # Check if the cop is about to collide with the border
            if self.is_about_to_collide(self.cop_pose):
                # Change direction if about to collide
                self.cop_speed *= -1

            # Calculate the angle to the thief
            angle_to_thief = math.atan2(self.thief_pose.y - self.cop_pose.y,
                                        self.thief_pose.x - self.cop_pose.x)
            cop_msg = Twist()
            cop_msg.linear.x = self.cop_speed
            cop_msg.angular.z = 4.0 * (angle_to_thief - self.cop_pose.theta)
            self.pub_cop.publish(cop_msg)

            # Check if the thief is about to collide with the border
            if self.is_about_to_collide(self.thief_pose):
                # Change direction if about to collide
                self.thief_speed *= -1

            # Calculate the distance between cop and thief
            distance_to_thief = math.sqrt((self.thief_pose.x - self.cop_pose.x)**2 + (self.thief_pose.y - self.cop_pose.y)**2)

            # If the distance is less than a certain threshold, the cop has caught the thief
            if distance_to_thief < 0.5:  # You can adjust this threshold as needed
                self.get_logger().info('Thief was caught at coordinates (x=%.2f, y=%.2f)'%(self.thief_pose.x, self.thief_pose.y))
                return  # Stop further processing if the thief is caught

            # Random movement for the thief
            thief_msg = Twist()
            thief_msg.linear.x = self.thief_speed
            thief_msg.angular.z = 4.0 * (self.thief_pose.theta + 0.1)  # Simple evasion logic
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



"""#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, Pose
from turtlesim.srv import Spawn
import math
import random
import threading
import time

cop_pos = [2.0, 2.0, 0.0]  # Initial position and orientation of cop turtle [x, y, theta]
thief_pos = [8.0, 8.0]  # Initial position of thief turtle [x, y]
cop_pub = None
thief_pub = None
node = None  # Initialize node globally

def spawn_turtles(node):
    global cop_pub, thief_pub

    # Create a client to call the /spawn service
    spawn_client = node.create_client(Spawn, 'spawn')

    # Wait for the /spawn service to be available
    while not spawn_client.wait_for_service(timeout_sec=1.0):
        print('Service "/spawn" not available, waiting...')

    # Create requests to spawn turtles
    cop_request = Spawn.Request()
    cop_request.name = 'cop_turtle'
    cop_request.x = cop_pos[0]  # Initial x position of cop turtle
    cop_request.y = cop_pos[1]  # Initial y position of cop turtle
    cop_request.theta = cop_pos[2]  # Initial orientation of cop turtle

    thief_request = Spawn.Request()
    thief_request.name = 'thief_turtle'
    thief_request.x = thief_pos[0]  # Initial x position of thief turtle
    thief_request.y = thief_pos[1]  # Initial y position of thief turtle
    thief_request.theta = 0.0  # Initial orientation of thief turtle

    # Call the /spawn service to spawn turtles
    cop_future = spawn_client.call_async(cop_request)
    thief_future = spawn_client.call_async(thief_request)

    # Wait for the spawn requests to be completed
    rclpy.spin_until_future_complete(node, cop_future)
    rclpy.spin_until_future_complete(node, thief_future)

    # Check if the turtles were successfully spawned
    if cop_future.result() is not None:
        print('Cop turtle spawned successfully')
    else:
        print('Failed to spawn cop turtle')

    if thief_future.result() is not None:
        print('Thief turtle spawned successfully')
    else:
        print('Failed to spawn thief turtle')

def cop_pose_callback(msg):
    global cop_pos
    cop_pos[0] = msg.x
    cop_pos[1] = msg.y
    cop_pos[2] = msg.theta
    print('Cop pose updated:', cop_pos)

def thief_pose_callback(msg):
    global thief_pos
    thief_pos[0] = msg.x
    thief_pos[1] = msg.y
    print('Thief pose updated:', thief_pos)

def move_cop():
    global cop_pos, thief_pos
    global cop_pub, node
	
    while rclpy.ok():
        print('Cop Position:', cop_pos)
        print('Thief Position:', thief_pos)

        distance = math.sqrt((cop_pos[0] - thief_pos[0]) ** 2 + (cop_pos[1] - thief_pos[1]) ** 2)
        print('Distance between cop and thief:', distance)

        dx = thief_pos[0] - cop_pos[0]
        dy = thief_pos[1] - cop_pos[1]
        direction = math.atan2(dy, dx)
        print('Direction to thief:', direction)

        angular_velocity = (direction - cop_pos[2]) * 0.5
        print('Adjusted angular velocity:', angular_velocity)

        cop_msg = Twist()
        cop_msg.linear.x = 1.0  # Set linear velocity to move forward
        cop_msg.angular.z = angular_velocity
        
        print('Cop Twist: ', cop_msg)

        if cop_pub is not None:
            cop_pub.publish(cop_msg)
            print('Cop Twist published:', cop_msg)
        else:
            print('Cop publisher not initialized')

        time.sleep(0.1)

def move_thief():
    global cop_pos, thief_pos
    global thief_pub, node

    while rclpy.ok():
        distance = math.sqrt((cop_pos[0] - thief_pos[0]) ** 2 + (cop_pos[1] - thief_pos[1]) ** 2)
        if distance <= 150:
            thief_speed = 20.0  # Ensure thief_speed is a float
        else:
            thief_speed = 5.0

        direction = random.uniform(0, 2 * math.pi)
        thief_pos[0] += math.cos(direction) * thief_speed * 0.1  # Update thief's x position
        thief_pos[1] += math.sin(direction) * thief_speed * 0.1  # Update thief's y position

        thief_msg = Twist()
        thief_msg.linear.x = thief_speed
        thief_msg.angular.z = 0.0
        
        print('Thief Twist:', thief_msg)

        if thief_pub is not None:
            thief_pub.publish(thief_msg)
            print('Thief Twist published:', thief_msg)
        else:
            print('Thief publisher not initialized')

        time.sleep(0.1)

def main():
    global cop_pub, thief_pub, node

    rclpy.init()
    node = rclpy.create_node('cop_and_thief_simulator')

    # Subscribe to pose topics to get updated positions of turtles
    cop_sub = node.create_subscription(Pose, '/cop_turtle/pose', cop_pose_callback, 10)
    thief_sub = node.create_subscription(Pose, '/thief_turtle/pose', thief_pose_callback, 10)

    # Spawn turtles
    spawn_turtles(node)

    cop_pub = node.create_publisher(Twist, '/cop_turtle/cmd_vel', 10)
    thief_pub = node.create_publisher(Twist, '/thief_turtle/cmd_vel', 10)

    cop_thread = threading.Thread(target=move_cop)
    thief_thread = threading.Thread(target=move_thief)

    cop_thread.start()
    thief_thread.start()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
