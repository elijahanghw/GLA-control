import time
import tqdm
import numpy as np
import control
import matplotlib.pyplot as plt
import scipy.io as sio

from matrices import *
from genetic import *
from timesim import *
from utils import *
from network import *

if __name__ == '__main__':
    ## Sim parameters
    num_chord = 8
    ref_chord = 0.2743
    V_inf = 10
    ds = 1/num_chord
    dt = ds*ref_chord/V_inf

    simulation_time = 4
    timesteps = int(simulation_time/dt)

    ## Load matrices
    A, B, C, D, T_rom, Ti_rom = system_mat()
    H5gust, H10gust, H15gust, H20gust, GustA, GustB, GustC, GustD = gusts()
    num_states = A.shape[0]
    sys = control.ss(A, B, C, D, dt)

    ## Open loop simulation
    T, plunge, pitch, bend = open_time_march(A, B, C, D, timesteps, dt, H10gust)

    ## GE for nn control
    generations = 500
    ga = Genetic(3, 3, num_pop=100)
    ga.initialize()
    fitness_history = np.zeros(generations)

    print("Starting genetic algorithm...")
    start_time = time.time()
    for gen in tqdm.tqdm(range(generations), desc="Generation", position=0):
        gen_time = time.time()
        for i, species in enumerate(ga.population):
            controller = Controller(individual=species)
            controller.setup()
            # Simulate and compute fitness
            overshoot_index = 0
            settling_index = 0
            input_index = 0
            fitness = 0
            for g in [H10gust, H20gust, GustA, GustC, GustD]:
                T, plunge_close, pitch_close, bend_close, i1, i2, i3 = close_time_march(A, B, C, D, controller, timesteps, dt, g)

                overshoot_index += -max(abs(plunge_close)) - 5*max(abs(pitch_close)) - 50000*max(abs(bend_close))
                    
                settling_index += -np.sum(T*(abs(plunge_close) + abs(10*pitch_close) + abs(5000*bend_close)))

                input_index += -max(abs(i1)) - max(abs(i2)) - max(abs(i3))

                fitness += overshoot_index + settling_index + input_index
            ga.fitness[i] = (species, fitness)
        ga.next_gen()
        fitness_history[gen] = ga.fitness_sorted[0][1]
        tqdm.tqdm.write(f"Generation: {gen+1}       Fitness: {ga.fitness_sorted[0][1]}")
            
    ## Optimal weights after GA
    optimal_individual = ga.fitness_sorted[0][0]
    optimal_controller = Controller(individual=optimal_individual)
    optimal_controller.setup()
    ## Close loop simluation
    T, plunge_close, pitch_close, bend_close, input1, input2, input3 = close_time_march(A, B, C, D, optimal_controller, timesteps, dt, H10gust)

    optimal_controller.save("GANN/network_weights.mat")
    sio.savemat("GANN/fitness.mat", mdict={"fitness": fitness_history})

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
