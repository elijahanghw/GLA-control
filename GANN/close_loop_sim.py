<<<<<<< HEAD
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

    simulation_time = 10
    timesteps = int(simulation_time/dt)

    ## Load matrices
    A, B, C, D, T_rom, Ti_rom = system_mat()
    H5gust, H10gust, H15gust, H20gust, GustA, GustB, GustC, GustD = gusts()
    num_states = A.shape[0]
    sys = control.ss(A, B, C, D, dt)

    gust_select = GustD

    ## Open loop simulation
    T, plunge, pitch, bend = open_time_march(A, B, C, D, timesteps, dt, gust_select)


            
    ## Load Controller
    controller = Controller()
    controller.load("GANN/trained_model/model3/network_weights.mat")

    ## Close loop simluation
    T, plunge_close, pitch_close, bend_close, input1, input2, input3 = close_time_march(A, B, C, D, controller, timesteps, dt, gust_select)


    ## Plot results

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

=======
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

    simulation_time = 10
    timesteps = int(simulation_time/dt)

    ## Load matrices
    A, B, C, D, T_rom, Ti_rom = system_mat()
    H5gust, H10gust, H15gust, H20gust, GustA, GustB, GustC, GustD = gusts()
    num_states = A.shape[0]
    sys = control.ss(A, B, C, D, dt)

    gust_select = H10gust

    ## Open loop simulation
    T, plunge, pitch, bend = open_time_march(A, B, C, D, timesteps, dt, gust_select)

            
    ## Load Controller
    controller = Controller()
    controller.load("GANN/trained_model/model3/network_weights.mat")

    ## Close loop simluation
    T, plunge_close, pitch_close, bend_close, input1, input2, input3 = close_time_march(A, B, C, D, controller, timesteps, dt, gust_select)


    ## Save NN Jacobian
    jacobian = controller.jacobian()
    sio.savemat("jacobian.mat", mdict={"jacobian": jacobian})

    ## Plot results

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

>>>>>>> c339e5e208710fac150003b1e3154d27225865e9
    plt.show()