import numpy as np
import control

from timesim import *

def log10uniform(low=0, high=1, size=None):
    return np.power(10 , np.random.uniform(low, high, size))

def ga_lqr(i, species, ga, num_states, sys, A, B, C, D, gust, timesteps, dt):
    # Define weighing matrices from genes
    Q = np.diag(species[:num_states])
    R = np.diag(species[num_states:])

    # Compute gain matrix
    K, S, E = control.lqr(sys, Q, R)
    E = np.log(E)/dt

    # Simulate and compute fitness
    T, plunge_close, pitch_close, bend_close, _, _, _ = close_time_march(A, B, C, D, K, timesteps, dt, gust)

    SI = -max(np.real(E))
    max_overshoot = 1*max(abs(plunge_close)) + 100*max(abs(pitch_close)) + 1000*max(abs(bend_close))

    fitness = 10*SI - max_overshoot # + -np.sum(T*(plunge_close**2 + (100*pitch_close)**2 + (1000*bend_close)**2))
    ga.fitness[i] = (species, fitness)