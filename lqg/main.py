import numpy as np
import multiprocessing as mp
from multiprocessing import Pool
import control
import matplotlib.pyplot as plt
import scipy.io as sio

from matrices import *
from genetic import *
from timesim import *
from utils import *

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
T, plunge, pitch, bend = open_time_march(A, B, C, D, timesteps, dt, H5gust)

## GE for lqr control
generations = 100
ga = Genetic(num_states, 3, num_pop=100)
ga.initialize()
fitness_history = np.zeros(generations)

print("Starting genetic algorithm...")

if __name__ == '__main__':
    for gen in range(generations):
        for i, species in enumerate(ga.population):
            # Define weighing matrices from genes
            Q = np.diag(species[:num_states])
            R = np.diag(species[num_states:])

            # Compute gain matrix
            K, S, E = control.lqr(sys, Q, R)
            E = np.log(E)/dt

            # Simulate and compute fitness
            T, plunge_close, pitch_close, bend_close, i1, i2, i3 = close_time_march(A, B, C, D, K, timesteps, dt, H5gust)

            SI = -max(np.real(E))
            overshoot_index = -max(abs(plunge_close)) - 100*max(abs(pitch_close)) - 5000*max(abs(bend_close))
            
            settling_index = -np.sum(T*(plunge_close**2 + (10*pitch_close)**2 + (200*bend_close)**2))

            input_index = -max(abs(i1)) - max(abs(i2)) - max(abs(i3))

            fitness = 10*SI + 10*overshoot_index + 0.1*settling_index + 30*input_index
            ga.fitness[i] = (species, fitness)

        ga.next_gen()
        fitness_history[gen] = ga.fitness_sorted[0][1]
        print(f"Generation: {gen+1}       Fitness: {ga.fitness_sorted[0][1]}")

## Optimal weights after GA
optimal_individual = ga.fitness_sorted[0][0]
Q_optimal = np.diag(optimal_individual[:num_states])
R_optimal = np.diag(optimal_individual[num_states:])
# Compute gain matrix
K_optimal, S, E = control.lqr(sys, Q_optimal, R_optimal)

## Save optimal weights and gains
sio.savemat("lqg/Q_optimal.mat", mdict={"Q_optimal": Q_optimal})
sio.savemat("lqg/R_optimal.mat", mdict={"R_optimal": R_optimal})
sio.savemat("lqg/K_optimal.mat", mdict={"K_optimal": K_optimal})

## Close loop simluation
T, plunge_close, pitch_close, bend_close, input1, input2, input3 = close_time_march(A, B, C, D, K_optimal, timesteps, dt, H5gust)

## Plot results
plt.figure(1)
plt.plot(range(1,generations+1), fitness_history)
plt.xlabel("Generations")
plt.ylabel("Fitness")

plt.figure(2)
plt.plot(T, plunge, 'k')
plt.plot(T, plunge_close, '--r')
plt.legend(("Open loop", "Close loop"))

plt.figure(3)
plt.plot(T, pitch/np.pi*180, 'k')
plt.plot(T, pitch_close/np.pi*180, '--r')
plt.legend(("Open loop", "Close loop"))

plt.figure(4)
plt.plot(T, bend*200, 'k')
plt.plot(T, bend_close*200, '--r')
plt.legend(("Open loop", "Close loop"))

plt.figure(5)
plt.plot(T, input1/np.pi*180, 'k')
plt.plot(T, input2/np.pi*180, 'b')
plt.plot(T, input3/np.pi*180, 'r')
plt.legend(("CS1", "CS2", "CS3"))

plt.show()
