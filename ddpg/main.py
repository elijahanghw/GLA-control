import sys
import time
import tqdm
import numpy as np
import pandas as pd
import scipy.io as sio
import matplotlib.pyplot as plt

from agent import Agent
from utils import *
from environment import Environment

## Sim parameters
num_chord = 8
ref_chord = 0.2743
V_inf = 10
ds = 1/num_chord
dt = ds*ref_chord/V_inf

simulation_time = 4
timesteps = int(simulation_time/dt)

env = Environment()

agent = Agent(env)
noise = OUNoise(env.norm_action_space, dt)

batch_size = 32
rewards = []
avg_rewards = []

num_episodes = 500
for episode in tqdm.tqdm(range(num_episodes), desc='Episode', position=0):
    start_time = time.time()
    state = env.reset()
    noise.reset()
    episode_reward = 0

    for step in tqdm.tqdm(range(timesteps), desc="Simulation", position=1, leave=False):
        action = agent.get_action(state)
        action = noise.get_action(action, step)
        new_state, reward = env.step(action)
        done = 1
        if step == timesteps-1:
            done = 0
        agent.memory.push(state, action, reward, new_state, done)
        state = new_state

        if len(agent.memory) > batch_size:
            agent.update(batch_size)

        episode_reward += reward
        
    rewards.append(episode_reward/1000)
    avg_rewards.append(np.mean(rewards[-20:]))
    tqdm.tqdm.write(f"Episode: {episode}      Reward: {episode_reward/1000}     Trailing 20 avg: {np.mean(rewards[-20:])} ")

# Save model
agent.save_actor_critic()
sio.savemat("ddpg/rewards.mat", mdict={"rewards": rewards, "avg_rewards": avg_rewards})
# Plot Rewards
plt.plot(rewards)
plt.plot(avg_rewards)
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.show()