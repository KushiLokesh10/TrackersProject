import pygame
import threading
import random
import math

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
COP_COLOR = (0, 0, 255)  # Blue
THIEF_COLOR = (255, 0, 0)  # Red
OBSTACLE_COLOR = (128, 128, 128)  # Gray
COP_SPEED = 3
THIEF_SPEED = 3
FONT_SIZE = 20
AVOID_RADIUS = 100  # Radius for obstacle avoidance
MIN_DISTANCE = 150  # Minimum distance to maintain from cop

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, FONT_SIZE)

# Cop class
class Cop(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.Surface((20, 20))
        self.image.fill(COP_COLOR)
        self.rect = self.image.get_rect(center=(random.randint(100, SCREEN_WIDTH - 100), random.randint(100, SCREEN_HEIGHT - 100)))
        self.velocity = pygame.math.Vector2(0, 0)

    def update(self, target_pos, obstacles):
        # Calculate desired velocity towards the target
        desired_velocity = pygame.math.Vector2(target_pos) - pygame.math.Vector2(self.rect.center)
        desired_velocity_length = desired_velocity.length()
        if desired_velocity_length > 0:
            desired_velocity.normalize_ip()
            desired_velocity *= COP_SPEED

        # Calculate steering force for obstacle avoidance
        avoidance_force = self.avoid_obstacles(obstacles)

        # Apply steering force to adjust velocity
        self.velocity += avoidance_force
        self.velocity += desired_velocity
        self.velocity.scale_to_length(min(COP_SPEED, self.velocity.length()))

        # Update position based on velocity
        new_rect = self.rect.move(self.velocity)
        collision = self.check_collision(new_rect, obstacles)
        if collision:
            new_rect = self.adjust_position(new_rect, collision)

        # Update position
        self.rect = new_rect
        self.rect.clamp_ip(screen.get_rect())

    def avoid_obstacles(self, obstacles):
        avoidance_force = pygame.math.Vector2(0, 0)
        for obstacle in obstacles:
            to_obstacle = pygame.math.Vector2(obstacle.rect.center) - pygame.math.Vector2(self.rect.center)
            distance = to_obstacle.length()
            if distance < AVOID_RADIUS:
                # Calculate steering force away from the obstacle
                avoidance_force -= to_obstacle.normalize() * (AVOID_RADIUS - distance)

        # Avoid screen edges as obstacles
        # Left edge
        if self.rect.left < AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(1, 0)
        # Right edge
        if self.rect.right > SCREEN_WIDTH - AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(-1, 0)
        # Top edge
        if self.rect.top < AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(0, 1)
        # Bottom edge
        if self.rect.bottom > SCREEN_HEIGHT - AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(0, -1)

        # Improved corner avoidance
        # Top-left corner
        if self.rect.left < AVOID_RADIUS and self.rect.top < AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(1, 1)
        # Top-right corner
        if self.rect.right > SCREEN_WIDTH - AVOID_RADIUS and self.rect.top < AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(-1, 1)
        # Bottom-left corner
        if self.rect.left < AVOID_RADIUS and self.rect.bottom > SCREEN_HEIGHT - AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(1, -1)
        # Bottom-right corner
        if self.rect.right > SCREEN_WIDTH - AVOID_RADIUS and self.rect.bottom > SCREEN_HEIGHT - AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(-1, -1)

        for obstacle in obstacles:
            to_obstacle = pygame.math.Vector2(obstacle.rect.center) - pygame.math.Vector2(self.rect.center)
            distance = to_obstacle.length()
            if distance < AVOID_RADIUS:
                avoidance_force -= to_obstacle.normalize() * (AVOID_RADIUS - distance)

        return avoidance_force

    def check_collision(self, new_rect, obstacles):
        for obstacle in obstacles:
            if new_rect.colliderect(obstacle.rect):
                return obstacle.rect
        return None

    def adjust_position(self, new_rect, collision):
        move_dir = pygame.math.Vector2(new_rect.center) - pygame.math.Vector2(collision.center)
        move_dir.normalize_ip()
        new_rect.center = collision.center + move_dir * (collision.width + 1)
        return new_rect

# Thief class
class Thief(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.Surface((20, 20))
        self.image.fill(THIEF_COLOR)
        self.rect = self.image.get_rect(center=(random.randint(100, SCREEN_WIDTH - 100), random.randint(100, SCREEN_HEIGHT - 100)))
        self.velocity = pygame.math.Vector2(0, 0)
        self.caught = False  # Initialize caught attribute

    def update(self, target_pos, obstacles):
        if not self.caught:
            # Calculate desired velocity away from the target (cop)
            desired_velocity = pygame.math.Vector2(self.rect.center) - pygame.math.Vector2(target_pos)
            desired_velocity_length = desired_velocity.length()
            if desired_velocity_length > 0 and desired_velocity_length < MIN_DISTANCE:
                desired_velocity.normalize_ip()
                desired_velocity *= THIEF_SPEED
            else:
                # Random movement if cop is not nearby
                desired_velocity = pygame.math.Vector2(random.randint(-1, 1), random.randint(-1, 1))
                if desired_velocity.length() > 0:
                    desired_velocity.normalize_ip()
                    desired_velocity *= THIEF_SPEED

            # Calculate steering force for obstacle avoidance
            avoidance_force = self.avoid_obstacles(obstacles)

            # Apply steering force to adjust velocity
            self.velocity += avoidance_force
            self.velocity += desired_velocity

            # Ensure velocity length is not zero before scaling
            if self.velocity.length() > 0:
                self.velocity.scale_to_length(min(THIEF_SPEED, self.velocity.length()))

            # Update position based on velocity
            new_rect = self.rect.move(self.velocity)
            collision = self.check_collision(new_rect, obstacles)
            if collision:
                new_rect = self.adjust_position(new_rect, collision)

            # Update position
            self.rect = new_rect
            self.rect.clamp_ip(screen.get_rect())

    def avoid_obstacles(self, obstacles):
        avoidance_force = pygame.math.Vector2(0, 0)
        for obstacle in obstacles:
            to_obstacle = pygame.math.Vector2(obstacle.rect.center) - pygame.math.Vector2(self.rect.center)
            distance = to_obstacle.length()
            if distance < AVOID_RADIUS:
                # Calculate steering force away from the obstacle
                avoidance_force -= to_obstacle.normalize() * (AVOID_RADIUS - distance)

        # Avoid screen edges as obstacles
        # Left edge
        if self.rect.left < AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(1, 0)
        # Right edge
        if self.rect.right > SCREEN_WIDTH - AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(-1, 0)
        # Top edge
        if self.rect.top < AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(0, 1)
        # Bottom edge
        if self.rect.bottom > SCREEN_HEIGHT - AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(0, -1)

        # Improved corner avoidance
        # Top-left corner
        if self.rect.left < AVOID_RADIUS and self.rect.top < AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(1, 1)
        # Top-right corner
        if self.rect.right > SCREEN_WIDTH - AVOID_RADIUS and self.rect.top < AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(-1, 1)
        # Bottom-left corner
        if self.rect.left < AVOID_RADIUS and self.rect.bottom > SCREEN_HEIGHT - AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(1, -1)
        # Bottom-right corner
        if self.rect.right > SCREEN_WIDTH - AVOID_RADIUS and self.rect.bottom > SCREEN_HEIGHT - AVOID_RADIUS:
            avoidance_force += pygame.math.Vector2(-1, -1)

        for obstacle in obstacles:
            to_obstacle = pygame.math.Vector2(obstacle.rect.center) - pygame.math.Vector2(self.rect.center)
            distance = to_obstacle.length()
            if distance < AVOID_RADIUS:
                avoidance_force -= to_obstacle.normalize() * (AVOID_RADIUS - distance)

        return avoidance_force

    def check_collision(self, new_rect, obstacles):
        for obstacle in obstacles:
            if new_rect.colliderect(obstacle.rect):
                return obstacle.rect
        return None

    def adjust_position(self, new_rect, collision):
        move_dir = pygame.math.Vector2(new_rect.center) - pygame.math.Vector2(collision.center)
        move_dir.normalize_ip()
        new_rect.center = collision.center + move_dir * (collision.width + 1)
        return new_rect

# Obstacle class
class Obstacle(pygame.sprite.Sprite):
    def __init__(self, x, y, width, height):
        super().__init__()
        self.image = pygame.Surface((width, height))
        self.image.fill(OBSTACLE_COLOR)
        self.rect = self.image.get_rect(topleft=(x, y))

# Brain thread function
def brain_thread(cop, thief):
    caught_position = None
    while True:
        # Cop behavior: Minimize distance to thief
        cop_target_pos = thief.rect.center
        cop.update(cop_target_pos, obstacles)

        # Thief behavior: Maximize distance from cop
        thief_target_pos = cop.rect.center
        thief.update(thief_target_pos, obstacles)

        # Check if cop caught the thief
        if not thief.caught and cop.rect.colliderect(thief.rect):
            caught_position = thief.rect.center
            thief.caught = True
            print("Thief caught at:", caught_position)
            break

        # Print coordinates
        cop_coordinates = cop.rect.center
        thief_coordinates = thief.rect.center
        print("Cop:", cop_coordinates)
        print("Thief:", thief_coordinates)

        # Delay to simulate real-time behavior
        pygame.time.delay(50)

# Create sprites
cop = Cop()
thief = Thief()

# Create obstacles
obstacles = [
    Obstacle(200, 200, 50, 50),
    Obstacle(400, 300, 100, 50),
    Obstacle(600, 100, 50, 150)
]

# Create sprite groups
all_sprites = pygame.sprite.Group([cop, thief] + obstacles)

# Start brain thread
brain_thread = threading.Thread(target=brain_thread, args=(cop, thief))
brain_thread.start()

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Draw everything
    screen.fill((255, 255, 255))
    all_sprites.draw(screen)

    # Draw coordinates
    cop_coordinates_text = font.render(f"Cop: {cop.rect.center}", True, (0, 0, 0))
    thief_coordinates_text = font.render(f"Thief: {thief.rect.center}", True, (0, 0, 0))
    screen.blit(cop_coordinates_text, (10, 10))
    screen.blit(thief_coordinates_text, (10, 10 + FONT_SIZE + 5))

    pygame.display.flip()

    clock.tick(60)

pygame.quit()
