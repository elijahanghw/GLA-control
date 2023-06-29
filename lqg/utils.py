import numpy as np
import control

from timesim import *

def log10uniform(low=0, high=1, size=None):
    return np.power(10 , np.random.uniform(low, high, size))

def ga_lqr(iterable, constant):
    i = iterable[0]
    species = iterable[1]
    ga = constant[0]
    num_states = constant[1]
    sys = constant[2]
    A = constant[3]
    B = constant[4]
    C = constant[5]
    D = constant[6]
    gust = constant[7]
    timesteps = constant[8]
    dt = constant[9]
    
    # Define weighing matrices from genes
    Q = np.diag(species[:num_states])
    R = np.diag(species[num_states:])

    # Compute gain matrix
    K, S, E = control.lqr(sys, Q, R)
    E = np.log(E)/dt

    # Simulate and compute fitness
    T, plunge_close, pitch_close, bend_close, i1, i2, i3 = close_time_march(A, B, C, D, K, timesteps, dt, gust)

    SI = -max(np.real(E))
    overshoot_index = -max(abs(plunge_close)) - 100*max(abs(pitch_close)) - 5000*max(abs(bend_close))         
    settling_index = -np.sum(T*(plunge_close**2 + (10*pitch_close)**2 + (200*bend_close)**2))
    input_index = -max(abs(i1)) - max(abs(i2)) - max(abs(i3))
    fitness = 10*SI + 10*overshoot_index + 0.1*settling_index + 30*input_index

    return (species, fitness)