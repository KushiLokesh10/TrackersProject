import pygame
import sys
import math
from pygame.locals import *
import threading
import time
import random
from pathfinding.core.diagonal_movement import DiagonalMovement
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

obstacles = [(300, 200, 20, 200), (400, 200, 30, 200), (200, 500, 120, 10), (500, 300, 70, 70),
             (0, 0, WIDTH, 10), (0, 0, 10, HEIGHT), (0, HEIGHT - 10, WIDTH, 10), (WIDTH - 10, 0, 10, HEIGHT)]

obstacle_rects = [pygame.Rect(obstacle) for obstacle in obstacles]

def move_cop():
    global cop_pos, thief_pos, obstacle_rects
    while True:
        distance = math.sqrt((cop_pos[0] - thief_pos[0]) ** 2 + (cop_pos[1] - thief_pos[1]) ** 2)
        if distance > 20:
            dx = thief_pos[0] - cop_pos[0]
            dy = thief_pos[1] - cop_pos[1]
            direction = math.atan2(dy, dx)
            next_x = cop_pos[0] + int(math.cos(direction) * 10)
            next_y = cop_pos[1] + int(math.sin(direction) * 10)
            next_x = max(0, min(WIDTH, next_x))
            next_y = max(0, min(HEIGHT, next_y))
            if not any(rect.collidepoint(next_x, next_y) for rect in obstacle_rects):
                cop_pos[0] = next_x
                cop_pos[1] = next_y
            else:
                dx = thief_pos[0] - cop_pos[0]
                dy = thief_pos[1] - cop_pos[1]
                direction = math.atan2(dy, dx)
                cop_pos[0] += int(math.cos(direction + math.pi / 2) * 10)
                cop_pos[1] += int(math.sin(direction + math.pi / 2) * 10)
        if distance <= 20:  # Check if the cop catches the thief
            print("Thief caught at position:", thief_pos)  # Print the position where the thief was caught
            escape_direction = random.uniform(0, 2 * math.pi)
            escape_x = thief_pos[0] + int(math.cos(escape_direction) * 50)
            escape_y = thief_pos[1] + int(math.sin(escape_direction) * 50)
            escape_x = max(0, min(WIDTH, escape_x))
            escape_y = max(0, min(HEIGHT, escape_y))
            if not any(rect.collidepoint(escape_x, escape_y) for rect in obstacle_rects):
                thief_pos[0] = escape_x
                thief_pos[1] = escape_y
            print("Thief escaped to position:", thief_pos)  # Print the new position of the thief
        print("Cop pos:", cop_pos)
        time.sleep(0.1)

def move_thief():
    global cop_pos, thief_pos, obstacles
    while True:
        distance = math.sqrt((cop_pos[0] - thief_pos[0]) ** 2 + (cop_pos[1] - thief_pos[1]) ** 2)
        if distance <= 200:
            thief_speed = 8
        else:
            thief_speed = 5

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
                # Move towards the path while maintaining some distance from cop
                for point in path:
                    if math.sqrt((cop_pos[0] - point[0]) ** 2 + (cop_pos[1] - point[1]) ** 2) >= 50:
                        thief_pos = point
                        direction = math.atan2(point[1] - thief_pos[1], point[0] - thief_pos[0])
                        break
        elif obstacle_detected and cop_detected:
            direction = random.uniform(0, 2 * math.pi)
        else:
            if 10 <= next_x <= WIDTH - 10 and 10 <= next_y <= HEIGHT - 10:
                thief_pos[0] = next_x
                thief_pos[1] = next_y
            else:
                direction = random.uniform(0, 2 * math.pi)
        print("Thief position:", thief_pos)
        time.sleep(0.1)

cop_thread = threading.Thread(target=move_cop)
thief_thread = threading.Thread(target=move_thief)
cop_thread.start()
thief_thread.start()
while True:
    screen.fill(BROWN)
    for obstacle in obstacles:
        pygame.draw.rect(screen, BLACK, pygame.Rect(obstacle))
    pygame.draw.circle(screen, BLUE, cop_pos, 10)
    pygame.draw.circle(screen, RED, thief_pos, 10)
    distance = math.sqrt((cop_pos[0] - thief_pos[0]) ** 2 + (cop_pos[1] - thief_pos[1]) ** 2)
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    pygame.display.flip()
