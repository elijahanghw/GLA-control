import numpy as np
import tqdm
import torch
import torch.autograd
import torch.optim as optim
import torch.nn as nn
import matplotlib.pyplot as plt

from networks import *
from agent import Agent
from environment import Environment

## Sim parameters
num_chord = 8
ref_chord = 0.2743
V_inf = 10
ds = 1/num_chord
dt = ds*ref_chord/V_inf

simulation_time = 2
timesteps = int(simulation_time/dt)

env = Environment()
agent = Agent(env)

agent.actor.load_state_dict(torch.load("ddpg/trained_models/ddpg1_cont/actor.pth"))

state = env.reset()
env.select_gust("A")
plunge = []
pitch = []
bend = []
input1 = []
input2 = []
input3 = []
rate1 = []
rate2 = []
rate3 = []
for step in tqdm.tqdm(range(timesteps), desc="Simulation", position=0):
    action = agent.get_action(state)
    new_state, _ = env.step(action)
    plunge.append(new_state[0])
    pitch.append(new_state[1])
    bend.append(new_state[2])
    input1.append(env.U[0])
    input2.append(env.U[1])
    input3.append(env.U[2])
    rate1.append(env.U_dot[0])
    rate2.append(env.U_dot[1])
    rate3.append(env.U_dot[2])
    state = new_state

# Plot results
plt.figure(1)
plt.plot(plunge)

plt.figure(2)
plt.plot(np.array(input1)*180/np.pi)
plt.plot(np.array(input2)*180/np.pi)
plt.plot(np.array(input3)*180/np.pi)
plt.legend(["CS1", "CS2", "CS3"])

plt.figure(3)
plt.plot(np.array(rate1)*180/np.pi)
plt.plot(np.array(rate2)*180/np.pi)
plt.plot(np.array(rate3)*180/np.pi)
plt.legend(["CS1", "CS2", "CS3"])
plt.show()
    