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

simulation_time = 5
timesteps = int(simulation_time/dt)

env = Environment()
agent = Agent(env)

agent.actor.load_state_dict(torch.load("ddpg/trained_models/ddpg2_fullstate/actor.pth"))

state = env.reset()
env.select_gust("H5")
plunge = []
pitch = []
bend = []
input1 = []
input2 = []
input3 = []
for step in tqdm.tqdm(range(timesteps), desc="Simulation", position=0):
    action = agent.get_action(state)
    new_state, _ = env.step(action)
    plunge.append(outputs[0])
    pitch.append(outputs[1])
    bend.append(outputs[2])
    input1.append(env.U[0])
    input2.append(env.U[1])
    input3.append(env.U[2])
    state = new_state

# Plot results
plt.figure(1)
plt.plot(bend)

plt.figure(2)
plt.plot(input1)
plt.plot(input2)
plt.plot(input3)
plt.legend(["CS1", "CS2", "CS3"])
plt.show()
    