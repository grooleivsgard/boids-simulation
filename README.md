# Boids Flocking Simulation

This repository contains a Python implementation of a boids flocking simulation using Pygame. The simulation demonstrates emergent flocking behavior (alignment, separation, and cohesion) as described in Craig Reynolds' classic Boids algorithm.

## Overview

The simulation initializes 200 boids that move around the screen while avoiding collisions, aligning with nearby boids, and staying together as a flock. The boids' behavior is governed by a combination of wandering, separation, alignment, and cohesion forces.

When you run the simulation, it will also capture each frame and save a GIF (`boids_simulation.gif`) of the simulation when you close the window.

## Demo

Check out the simulation in action:

![Boids Simulation](boids_simulation.gif)

*Click on the image for a larger view, if needed.*

## Requirements

- Python 3.x
- [Pygame](https://www.pygame.org/)
- [Imageio](https://imageio.readthedocs.io/)
- [NumPy](https://numpy.org/)

Install the required packages with:

```bash
pip install pygame imageio numpy


