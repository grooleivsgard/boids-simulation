import math
from random import randint, uniform
import pygame as pg
import imageio  # New import for saving GIFs
import numpy as np

# Color constants
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Window Parameters
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Parameters
NUM_BOIDS = 200
BOID_SIZE = 10
SPEED = 3.5
MAX_FORCE = 0.3
BOID_FRICTION = 0.75

WANDER_RADIUS = 30

SEPARATION = 1.2
SEPARATION_RADIUS = 40

ALIGNMENT = 1
ALIGNMENT_RADIUS = 50

COHESION = 1
COHESION_RADIUS = 80

class Simulation:

    def __init__(self):
        pg.init()
        self.running = False
        self.clock = pg.time.Clock()
        self.screen = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.screen_rect = self.screen.get_rect()
        self.fps = 60

        self.boids = []
        for i in range(NUM_BOIDS):
            self.boids.append(Boid(self, (randint(0, SCREEN_WIDTH), randint(0, SCREEN_HEIGHT))))

        self.frames = []  # List to store captured frames

    def events(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False

    def draw(self):
        # Clear the screen
        self.screen.fill(BLACK)

        # Draw all boids
        for boid in self.boids:
            boid.draw(self.screen)

        # Update the display
        pg.display.update()

        # Capture the current frame
        frame = pg.surfarray.array3d(self.screen)
        # Transpose the frame array from (width, height, channels) to (height, width, channels)
        frame = np.transpose(frame, (1, 0, 2))
        self.frames.append(frame)

    def update(self):
        """
        Advances the simulation by one step.
        """
        for boid in self.boids:
            boid.update()

    def run(self):
        self.running = True
        while self.running:
            self.clock.tick(self.fps)
            self.events()
            self.update()
            self.draw()
        # Simulation has ended; print message for debugging.
        print("Simulation ended, saving GIF...")
        gif_filename = "boids_simulation.gif"
        imageio.mimsave(gif_filename, self.frames, fps=self.fps)
        print(f"Simulation GIF saved as {gif_filename}")


# (Rest of your code remains the same)

class PhysicsObjet:
    def __init__(self, simulation, position):
        self.simulation = simulation
        self.acc = pg.math.Vector2(0, 0)
        self.vel = pg.math.Vector2(0, 0)
        self.pos = pg.math.Vector2(position)
        self.speed = 1
        self.friction = 0.9

    def update(self):
        self.vel += self.acc
        self.pos += self.vel * self.speed
        self.acc *= 0  # Reset acceleration
        self.vel *= self.friction  # Apply friction

        # Wrap around screen edges
        if self.pos.x > self.simulation.screen_rect.w:
            self.pos.x -= self.simulation.screen_rect.w
        elif self.pos.x < 0:
            self.pos.x += self.simulation.screen_rect.w

        if self.pos.y > self.simulation.screen_rect.h:
            self.pos.y -= self.simulation.screen_rect.h
        elif self.pos.y < 0:
            self.pos.y += self.simulation.screen_rect.h

class Boid(PhysicsObjet):
    def __init__(self, simulation, position):
        super().__init__(simulation, position)
        self.speed = SPEED
        self.vel = pg.math.Vector2(randint(-2, 2), randint(-2, 2))
        self.max_force = MAX_FORCE
        self.friction = BOID_FRICTION
        self.target = pg.math.Vector2(0, 0)
        self.future_loc = pg.math.Vector2(0, 0)
        self.theta = uniform(-math.pi, math.pi)

    def update(self):
        self.acc += self.wander()
        self.acc += self.separation() * SEPARATION
        self.acc += self.alignment() * ALIGNMENT
        self.acc += self.cohesion() * COHESION
        super().update()

    def separation(self):
        force_vector = pg.math.Vector2(0, 0)
        neighbors = 0
        for boid in self.simulation.boids:
            if boid == self:
                continue
            distance = self.pos.distance_to(boid.pos)
            if 0 < distance < SEPARATION_RADIUS:
                neighbors += 1
                steer_boid = self.pos - boid.pos
                scaled_distance = remap(distance, 0, SEPARATION_RADIUS, 1, 0)
                steer_boid *= scaled_distance
                force_vector += steer_boid
        if neighbors > 0:
            force_vector /= neighbors
            if force_vector.length() > 0:
                limit(force_vector, MAX_FORCE)
        return force_vector

    def alignment(self):
        force_vector = pg.math.Vector2(0, 0)
        neighbors = 0
        for boid in self.simulation.boids:
            if boid == self:
                continue
            distance = self.pos.distance_to(boid.pos)
            if 0 < distance < ALIGNMENT_RADIUS:
                neighbors += 1
                force_vector += boid.vel
        if neighbors > 0:
            force_vector /= neighbors
            force_vector = force_vector - self.vel
            if force_vector.length() > 0:
                limit(force_vector, MAX_FORCE)
        return force_vector

    def cohesion(self):
        force_vector = pg.math.Vector2(0, 0)
        neighbors = 0
        for boid in self.simulation.boids:
            if boid == self:
                continue
            distance = self.pos.distance_to(boid.pos)
            if 0 < distance < COHESION_RADIUS:
                neighbors += 1
                force_vector += boid.pos
        if neighbors > 0:
            force_vector /= neighbors
            force_vector -= self.pos
            if force_vector.length() > 0:
                limit(force_vector, MAX_FORCE)
        return force_vector

    def move_towards_target(self, target):
        desired = target - self.pos
        distance = desired.length()
        desired = desired.normalize()
        radius = 100
        if distance < radius:
            m = remap(distance, 0, radius, 0, self.speed)
            desired *= m
        else:
            desired *= self.speed
        force_vector = desired - self.vel
        limit(force_vector, self.max_force)
        return force_vector

    def wander(self):
        if self.vel.length_squared() != 0:
            self.future_loc = self.vel.normalize() * 80
            self.theta += uniform(-math.pi, math.pi) / 10
            self.target = self.pos + self.future_loc + pg.math.Vector2(WANDER_RADIUS * math.cos(self.theta),
                                                                       WANDER_RADIUS * math.sin(self.theta))
        return self.move_towards_target(self.target)

    def draw(self, screen):
        angle = math.atan2(self.vel.y, self.vel.x)
        other_points_angle = 0.75 * math.pi
        x0 = self.pos.x + BOID_SIZE * math.cos(angle)
        y0 = self.pos.y + BOID_SIZE * math.sin(angle)
        x1 = self.pos.x + BOID_SIZE * math.cos(angle + other_points_angle)
        y1 = self.pos.y + BOID_SIZE * math.sin(angle + other_points_angle)
        x2 = self.pos.x + BOID_SIZE * math.cos(angle - other_points_angle)
        y2 = self.pos.y + BOID_SIZE * math.sin(angle - other_points_angle)
        pg.draw.polygon(screen, WHITE, [(x1, y1), (x2, y2), (x0, y0)])

def remap(n, start1, stop1, start2, stop2):
    new_value = (n - start1) / (stop1 - start1) * (stop2 - start2) + start2
    if start2 < stop2:
        return constrain(new_value, start2, stop2)
    else:
        return constrain(new_value, stop2, start2)

def constrain(n, low, high):
    return max(min(n, high), low)

def limit(vector, length):
    if vector.length_squared() <= length * length:
        return
    else:
        vector.scale_to_length(length)

if __name__ == '__main__':
    sim = Simulation()
    sim.run()
