import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from agent import Agent
from utils import *
from environment import Environment

env = Environment()

agent = Agent(env)
noise = OUNoise(env.action_space)

batch_size = 128
rewards = []
avg_rewards = []