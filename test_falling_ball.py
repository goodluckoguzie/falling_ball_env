import gym
import falling_ball_env
import numpy as np

def test_falling_ball_env():
    env = gym.make('FallingBall-v0')
    num_episodes = 20


    # Print the min-max values
    env.print_observation_bounds()

    # Initialize variables to hold min and max observations
    min_obs = np.array([float('inf'), float('inf'), float('inf')])
    max_obs = np.array([float('-inf'), float('-inf'), float('-inf')])

    for episode in range(num_episodes):
        observation = env.reset()
        done = False
        timesteps = 0
        while not done:
            env.render()
            action = env.action_space.sample()  # Sample a random action (no-op in our case)

            # Update min and max observations
            min_obs = np.minimum(min_obs, observation)
            max_obs = np.maximum(max_obs, observation)

            observation, reward, done, info = env.step(action)
            timesteps += 1
            if done:
                print(f"Episode {episode + 1} finished after {timesteps} timesteps.")
                break

    # Print the min and max observations
    print("Minimum observed values: ", min_obs)
    print("Maximum observed values: ", max_obs)

    env.close()

if __name__ == "__main__":
    test_falling_ball_env()
