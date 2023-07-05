import time
import numpy as np
import multiprocessing as mp
import control
import matplotlib.pyplot as plt
import scipy.io as sio
from functools import partial

from matrices import *
from timesim import *

if __name__ == '__main__':
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

    ## PID controller
    K_p = 100

    ## Close loop simluation
    T, plunge_close, pitch_close, bend_close, input1 = close_time_march(A, B, C, D, K_p, timesteps, dt, H5gust)


    plt.figure(2)
    plt.plot(T, plunge, 'k')
    plt.plot(T, plunge_close, '--r')
    plt.legend(("Open loop", "Close loop"))
    plt.grid(True)

    plt.figure(3)
    plt.plot(T, pitch/np.pi*180, 'k')
    plt.plot(T, pitch_close/np.pi*180, '--r')
    plt.legend(("Open loop", "Close loop"))
    plt.grid(True)

    plt.figure(4)
    plt.plot(T, bend*200, 'k')
    plt.plot(T, bend_close*200, '--r')
    plt.legend(("Open loop", "Close loop"))
    plt.grid(True)

    plt.figure(5)
    plt.plot(T, input1/np.pi*180, 'k')
    plt.grid(True)

    plt.show()
