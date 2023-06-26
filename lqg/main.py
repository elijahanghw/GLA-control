import numpy as np
import control
import matplotlib.pyplot as plt

from matrices import *
from genetic import *

## Sim parameters
num_chord = 8
ref_chord = 0.2743
V_inf = 10
ds = 1/num_chord
dt = ds*ref_chord/V_inf

simulation_time = 5
timesteps = int(simulation_time/dt)

## Load matrices
A, B, C, D, T_rom, Ti_rom = system_mat()
H1gust, H2gust, H3gust, H5gust = gusts()

num_states = A.shape[0]

sys = control.ss(A, B, C, D, dt)

## Open loop simulation
# Initialization
x_old = np.zeros(num_states)
plunge = np.zeros(timesteps)
pitch = np.zeros(timesteps)
bend = np.zeros(timesteps)
T = np.zeros(timesteps)

U = np.array([0, 0, 0])

for t in range(timesteps):
    x_new = np.matmul(A, x_old) + np.matmul(B, U) + H5gust[:,t]
    y = np.matmul(C, x_new)
    plunge[t] = y[0]
    pitch[t] = y[1]
    bend[t] = y[2]
    T[t] = t*dt
    x_old = x_new

## GE for lqr control
ga = Genetic(num_states, 3, num_pop=10)
ga.initialize()
generations = 1000

for species in ga.population:
    # Define weighing matrices from genes
    Q = np.diag(species[:num_states])
    R = np.diag(species[num_states:])

    # Compute gain matrix
    K, S, E = control.lqr(sys, Q, R)

    # Simulate and compute fitness
    x_old_close = np.zeros(num_states)
    plunge_close = np.zeros(timesteps)
    pitch_close = np.zeros(timesteps)
    bend_close = np.zeros(timesteps)
    T = np.zeros(timesteps)

    for t in range(timesteps):
        x_new_close = np.matmul((A - np.matmul(B, K)), x_old_close) + H5gust[:,t]
        y = np.matmul(C, x_new_close)
        plunge_close[t] = y[0]
        pitch_close[t] = y[1]
        bend_close[t] = y[2]
        x_old_close = x_new_close 
        T[t] = t*dt

    fitness = -np.sum(10*T*(plunge_close**2 + pitch_close**2 + bend_close**2))
    ga.fitness.append((species, fitness))

ga.next_gen()

print(len(ga.population))


# ## Close loop simluation
# # Initialization
# x_old_close = np.zeros(num_states)
# plunge_close = np.zeros(timesteps)
# pitch_close = np.zeros(timesteps)
# bend_close = np.zeros(timesteps)
# input1 = np.zeros(timesteps)
# input2 = np.zeros(timesteps)
# input3 = np.zeros(timesteps)
# T = np.zeros(timesteps)

# for t in range(timesteps):
#     x_new_close = np.matmul((A - np.matmul(B, K)), x_old_close) + H5gust[:,t]
#     y = np.matmul(C, x_new_close)
#     plunge_close[t] = y[0]
#     pitch_close[t] = y[1]
#     bend_close[t] = y[2]
#     inputs = -np.matmul(K, x_old_close)
#     input1[t] = inputs[0]
#     input2[t] = inputs[1]
#     input3[t] = inputs[2]
#     T[t] = t*dt
#     x_old_close = x_new_close 

# plt.figure(1)
# plt.plot(T, plunge)
# plt.plot(T, plunge_close)

# plt.figure(2)
# plt.plot(T, pitch)
# plt.plot(T, pitch_close)

# plt.figure(3)
# plt.plot(T, bend)
# plt.plot(T, bend_close)

# plt.show()
