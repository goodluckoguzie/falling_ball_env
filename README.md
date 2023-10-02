
# Falling Ball Environment

A custom OpenAI Gym environment where a ball falls due to gravity and bounces off the floor.

## Overview

The environment simulates a ball falling due to gravity. The environment keeps track of the ball's position, its velocity, and ensures that the ball bounces off the floor with a reduced velocity, simulating energy loss.

## Features

- **Gravity Simulation**: The environment has a gravity factor that affects the ball's downward motion.
- **Bouncing Mechanism**: When the ball hits the floor, it bounces back with a reduced velocity.
- **Render Support**: The environment can be rendered to visualize the ball's motion.
  
## Getting Started

### Dependencies

- Python
- OpenAI Gym
- Numpy
- OpenCV

### Installation

<Your installation steps here, e.g., cloning the repo, installing dependencies>

### Usage

Here's a basic usage example:

```python
import gym
env = gym.make('FallingBall-v0')

for _ in range(10):  # Try 10 episodes
    obs = env.reset()
    done = False
    while not done:
        obs, reward, done, info = env.step(None)
        env.render()
