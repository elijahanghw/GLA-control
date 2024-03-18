import numpy as np
from network import *

def open_time_march(A, B, C, D, timesteps, dt, gust):
    num_states = A.shape[0]
    x_old = np.zeros(num_states)
    plunge = np.zeros(timesteps)
    pitch = np.zeros(timesteps)
    bend = np.zeros(timesteps)
    T = np.zeros(timesteps)

    U = np.array([0, 0, 0])

    for t in range(timesteps):
        x_new = np.matmul(A, x_old) + np.matmul(B, U) + gust[:,t]
        y = np.matmul(C, x_new)
        plunge[t] = y[0]
        pitch[t] = y[1]
        bend[t] = y[2]
        T[t] = t*dt
        x_old = x_new

    return T, plunge, pitch, bend

def close_time_march(A, B, C, D, controller, timesteps, dt, gust):
    num_states = A.shape[0]
    num_input = B.shape[1]
    x_old = np.zeros(num_states)
    y = np.zeros(3)
    plunge = np.zeros(timesteps)
    pitch = np.zeros(timesteps)
    bend = np.zeros(timesteps)
    input1 = np.zeros(timesteps)
    input2 = np.zeros(timesteps)
    input3 = np.zeros(timesteps)
    T = np.zeros(timesteps)

    for t in range(timesteps):
        U = controller.forward(y)
        x_new = np.matmul(A, x_old) + np.matmul(B, U) + gust[:,t]
        y = np.matmul(C, x_new)
        plunge[t] = y[0]
        pitch[t] = y[1]
        bend[t] = y[2]
        input1[t] = U[0]
        input2[t] = U[1]
        input3[t] = U[2]
        T[t] = t*dt
        x_old = x_new

    return T, plunge, pitch, bend, input1, input2, input3