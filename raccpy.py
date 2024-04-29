import pygame
import sys
import math
import threading
import time
import random
from pygame.locals import *
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.grid import Grid

pygame.init()
WIDTH = 800
HEIGHT = 600

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
BROWN = (222, 171, 144)

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Cop and Thief")

cop_pos = [200, 100]
thief_pos = [269, 370]
cop_speed_limit = 9
thief_before_speed= 5
thief_speed_limit = 7
# Set speed limit for the thief
angular_velocity = 1
caught = False

obstacles = [(0, 0, WIDTH, 10), (0, 0, 10, HEIGHT), (0, HEIGHT - 10, WIDTH, 10), (WIDTH - 10, 0, 10, HEIGHT)]
obstacle_rects = [pygame.Rect(obstacle) for obstacle in obstacles]
cop_rect = pygame.Rect(200, 100, 20, 20)  # Define cop as a rectangle
thief_rect = pygame.Rect(269, 370, 20, 20)
def move_cop():
    global cop_pos, thief_pos, caught
    while not caught:
        distance = math.sqrt((cop_pos[0] - thief_pos[0]) ** 2 + (cop_pos[1] - thief_pos[1]) ** 2)
        if distance > 20:
            dx = thief_pos[0] - cop_pos[0]
            dy = thief_pos[1] - cop_pos[1]
            direction = math.atan2(dy, dx)
            next_x = cop_pos[0] + int(math.cos(direction) * cop_speed_limit)
            next_y = cop_pos[1] + int(math.sin(direction) * cop_speed_limit)
            next_x = max(0, min(WIDTH, next_x))
            next_y = max(0, min(HEIGHT, next_y))
            if not any(rect.collidepoint(next_x, next_y) for rect in obstacle_rects):
                cop_pos[0] = next_x
                cop_pos[1] = next_y
            else:
                dx = thief_pos[0] - cop_pos[0]
                dy = thief_pos[1] - cop_pos[1]
                direction = math.atan2(dy, dx)
                cop_pos[0] += int(math.cos(direction + math.pi / 2) * cop_speed_limit)
                cop_pos[1] += int(math.sin(direction + math.pi / 2) * cop_speed_limit)
        print("Cop pos: ", cop_pos)
        obstacle_detected = any(
            rect.collidepoint(next_x, next_y)
            for rect in obstacle_rects
            if math.sqrt((cop_pos[0] - rect.centerx) ** 2 + (cop_pos[1] - rect.centery) ** 2) < 200
        )

        if not obstacle_detected:
            cop_pos[0] = next_x
            cop_pos[1] = next_y
        else:
            direction += random.uniform(-math.pi / 2, math.pi / 2)
            cop_pos[0] += int(math.cos(direction) * cop_speed_limit)
            cop_pos[1] += int(math.sin(direction) * cop_speed_limit)
        time.sleep(0.1)

def move_thief():
    global cop_pos, thief_pos, caught, angular_velocity
    while not caught:
        distance = math.sqrt((cop_pos[0] - thief_pos[0]) ** 2 + (cop_pos[1] - thief_pos[1]) ** 2)
        if distance <= 50:
            thief_speed = max(thief_speed_limit, distance)
            angular_velocity = 1
        else:
            thief_speed = thief_before_speed
            angular_velocity = 1

        if 'direction' not in locals():
            direction = random.uniform(0, 2 * math.pi)
        next_x = thief_pos[0] + int(math.cos(direction) * thief_speed)
        next_y = thief_pos[1] + int(math.sin(direction) * thief_speed)

        obstacle_detected = any(
            rect.collidepoint(next_x + int(math.cos(direction) * d),
                              next_y + int(math.sin(direction) * d))
            for d in range(10, 50, 10) for rect in obstacle_rects
        ) or any(
            (cop_pos[0] - 20 <= next_x + int(math.cos(direction) * d) <= cop_pos[0] + 20 and
             cop_pos[1] - 20 <= next_y + int(math.sin(direction) * d) <= cop_pos[1] + 20)
            for d in range(10, 50, 10)
        )

        cop_detected = obstacle_detected

        if obstacle_detected and not cop_detected:
            grid_data = [[0] * WIDTH for _ in range(HEIGHT)]
            for obstacle in obstacles:
                x, y, w, h = obstacle
                for i in range(y, y + h):
                    for j in range(x, x + w):
                        grid_data[i][j] = 1
            grid = Grid(matrix=grid_data)
            finder = AStarFinder()
            start_node = grid.node(*thief_pos)
            random_pos = (random.randint(0, WIDTH), random.randint(0, HEIGHT))
            end_node = grid.node(*random_pos)
            path, _ = finder.find_path(start_node, end_node, grid)
            if path:
                thief_pos = path[1]
                direction = math.atan2(path[1][1] - thief_pos[1], path[1][0] - thief_pos[0])
            else:
                direction = random.uniform(0, 2 * math.pi)
        elif cop_detected and not obstacle_detected:
            grid_data = [[0] * WIDTH for _ in range(HEIGHT)]
            for obstacle in obstacles:
                x, y, w, h = obstacle
                for i in range(y, y + h):
                    for j in range(x, x + w):
                        grid_data[i][j] = 1
            grid = Grid(matrix=grid_data)
            finder = AStarFinder()
            start_node = grid.node(*thief_pos)
            end_node = grid.node(*cop_pos)
            path, _ = finder.find_path(start_node, end_node, grid)
            if path:
                thief_pos = path[1]
                direction = math.atan2(path[1][1] - thief_pos[1], path[1][0] - thief_pos[0])
        elif obstacle_detected and cop_detected:
            direction = random.uniform(0, 2 * math.pi)
        else:
            if 10 <= next_x <= WIDTH - 10 and 10 <= next_y <= HEIGHT - 10:
                thief_pos[0] = next_x
                thief_pos[1] = next_y
                direction += random.uniform(-angular_velocity,
                                            angular_velocity)

            else:
                direction = random.uniform(0, 2 * math.pi)
        print("Thief position:", thief_pos)
        time.sleep(0.1)

cop_thread = threading.Thread(target=move_cop)
thief_thread = threading.Thread(target=move_thief)
cop_thread.start()
thief_thread.start()

while not caught:
    screen.fill(BROWN)
    for obstacle in obstacles:
        pygame.draw.rect(screen, BLACK, pygame.Rect(obstacle))
    # pygame.draw.rect(screen, RED, pygame.Rect(cop_pos[0] - 10, cop_pos[1] - 10, 20, 20))  # Draw cop as a red rectangle
    # pygame.draw.rect(screen, BLUE, pygame.Rect(thief_pos[0] - 10, thief_pos[1] - 10, 20, 20))
    pygame.draw.circle(screen , RED , cop_pos , 10)
    pygame.draw.circle(screen , BLUE , thief_pos , 10)
    distance = math.sqrt((cop_pos[0] - thief_pos[0]) ** 2 + (cop_pos[1] - thief_pos[1]) ** 2)
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    pygame.display.flip()

cop_thread.join()
thief_thread.join()

print("Simulation stopped. Thief caught at position:", thief_pos)
