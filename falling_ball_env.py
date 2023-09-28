import gym
from gym import spaces
import numpy as np
import cv2
import random

DEFAULT_RADIUS = 0.2
RANDOM_HEIGHT = True
VELOCITY_BOUNDS = 1.0  #  value

class Ball:
    def __init__(self, x, y, radius=DEFAULT_RADIUS, color=(0, 0, 255), velocity_y=0.0):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.velocity_y = velocity_y

    def draw(self, image, px_to_world, map_dim):
        cv2.circle(image, (self.w2px(self.x, px_to_world, map_dim), 
                           self.w2py(self.y, px_to_world, map_dim)), 
                   int(self.radius * px_to_world), 
                   self.color, -1)

    @staticmethod
    def w2px(value, px_to_world, map_dim):
        return int((value + map_dim / 2) * px_to_world)

    @staticmethod
    def w2py(value, px_to_world, map_dim):
        return int(map_dim * px_to_world - (value + map_dim / 2) * px_to_world)

class FallingBallEnv(gym.Env):
    def __init__(self):
        self.map_dim = 2.0
        self.gravity = 0.001
        self.time_step = 0.5 #0.05
        self.px_to_world = 200
        self.current_step = 0
        self.max_timesteps = 500 #1000

        self.action_space = spaces.Discrete(1)  # No-op action
        # self.observation_space = spaces.Box(low=0, high=1, shape=(2,), dtype=np.float32)
        # self.observation_space = spaces.Box(low=np.array([0, -self.map_dim / 2, -np.inf]),
        #                                     high=np.array([1, self.map_dim / 2, np.inf]), 
        #                                     shape=(3,), dtype=np.float32)

        self.observation_space = spaces.Box(low=np.array([0, -self.map_dim / 2, -VELOCITY_BOUNDS]),
                                            high=np.array([1, self.map_dim / 2, VELOCITY_BOUNDS]), 
                                            shape=(3,), dtype=np.float32)

        self.reset()

    def step(self, _):
        # Update the ball's position based on its current velocity
        self.ball.y += self.ball.velocity_y * self.time_step
        
        # Apply gravity to the ball's velocity
        self.ball.velocity_y -= self.gravity*0.8

        # Clamp the velocity to make sure it's within the predefined bounds
        self.ball.velocity_y = np.clip(self.ball.velocity_y, -VELOCITY_BOUNDS, VELOCITY_BOUNDS)

        # Bounce when hit the floor and reduce the velocity (simulate energy loss)
        if self.ball.y - self.ball.radius <= -self.map_dim / 2:
            self.ball.y = -self.map_dim / 2 + self.ball.radius
            self.ball.velocity_y = -self.ball.velocity_y * 0.8

        # Update the current step and check for termination
        self.current_step += 1
        done = self.current_step >= self.max_timesteps

        # Return the observation, reward, done status, and info
        return np.array([self.ball.x, self.ball.y, self.ball.velocity_y]), 0, done, {}

    # def step(self, _):
    #     self.ball.y += self.ball.velocity_y * self.time_step
    #     self.ball.velocity_y -= self.gravity*0.8

    #     # Bounce when hit the floor
    #     if self.ball.y - self.ball.radius <= -self.map_dim / 2:
    #         self.ball.y = -self.map_dim / 2 + self.ball.radius
    #         self.ball.velocity_y = -self.ball.velocity_y * 0.8

    #     self.current_step += 1
    #     done = self.current_step >= self.max_timesteps

    #     # return np.array([self.ball.x, self.ball.y]), 0, done, {}
    #     return np.array([self.ball.x, self.ball.y, self.ball.velocity_y]), 0, done, {}

    def reset(self):
        ball_radius = DEFAULT_RADIUS
        min_height = -self.map_dim / 2 + ball_radius
        if RANDOM_HEIGHT is False:
            initial_height = 1.0
        else:
            initial_height = random.random()*(1.-min_height)+min_height
        # print(f'initial height {initial_height}')
        self.ball = Ball(0.5, initial_height, radius=ball_radius, velocity_y=-0.02)


        self.current_step = 0
        # return np.array([self.ball.x, self.ball.y])
        return np.array([self.ball.x, self.ball.y, self.ball.velocity_y])

    def render(self, mode='human'):
        img = self.draw_environment([self.ball.x, self.ball.y])
        cv2.imshow('FallingBallEnv', img)
        cv2.waitKey(10)

    def close(self):
        cv2.destroyAllWindows()


    # def draw_environment(self, ball_obs):
    #     image = np.ones((int(self.map_dim * self.px_to_world), 
    #                      int(self.map_dim * self.px_to_world), 3), dtype=np.uint8) * 255  # White background
    
    #     # Draw the ball
    #     ball = Ball(ball_obs[0], ball_obs[1])
    #     ball.draw(image, self.px_to_world, self.map_dim)

    #     # Draw walls
    #     thickness = 3
    #     cv2.rectangle(image, (0, 0), (int(self.map_dim * self.px_to_world), 
    #                                   int(self.map_dim * self.px_to_world)), (0, 0, 0), thickness)
    
    #     # Draw the red floor
    #     cv2.line(image, (0, int(self.map_dim * self.px_to_world - thickness)), 
    #              (int(self.map_dim * self.px_to_world), int(self.map_dim * self.px_to_world - thickness)), 
    #              (0, 0, 255), 5*thickness)  # Red floor
    
    #     return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    def draw_environment(self, ball_obs):
        image = np.ones((int(self.map_dim * self.px_to_world), 
                        int(self.map_dim * self.px_to_world), 3), dtype=np.uint8) * 255
        ball = Ball(ball_obs[0], ball_obs[1], radius=0.2)
        ball.draw(image, self.px_to_world, self.map_dim)
        thickness = 3
        cv2.rectangle(image, (0, 0), (int(self.map_dim * self.px_to_world), 
                                    int(self.map_dim * self.px_to_world)), (0, 0, 0), thickness)
        cv2.line(image, (0, int(self.map_dim * self.px_to_world - thickness)), 
                (int(self.map_dim * self.px_to_world), int(self.map_dim * self.px_to_world - thickness)), 
                (0, 0, 255), 5*thickness)
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    def print_observation_bounds(self):
        low_bounds = self.observation_space.low
        high_bounds = self.observation_space.high

        print("Min bounds for normalization:", low_bounds)
        print("Max bounds for normalization:", high_bounds)

gym.envs.registration.register(
    id='FallingBall-v0',
    entry_point='falling_ball_env:FallingBallEnv',
)


# # To test the environment
# env = FallingBallEnv()
# for _ in range(10):  # Try 10 episodes
#     obs = env.reset()
#     done = False
#     while not done:
#         obs, reward, done, info = env.step(None)
#         print("obs",obs)
#         print("obs.shape",obs.shape)
#         env.render()
