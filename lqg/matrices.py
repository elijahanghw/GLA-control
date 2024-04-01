import scipy.io as sio
import numpy as np

def system_mat():
    A = sio.loadmat('model/reduced/A_rsys.mat')["A_rsys"]
    B = sio.loadmat('model/reduced/B_rsys.mat')["B_rsys"]
    C = sio.loadmat('model/reduced/C_rsys.mat')["C_rsys"]
    D = sio.loadmat('model/reduced/D_rsys.mat')["D_rsys"]

    T_rom = sio.loadmat('model/reduced/T_rom.mat')["T_rom"]
    Ti_rom = sio.loadmat('model/reduced/Ti_rom.mat')["Ti_rom"]

    return A, B, C, D, T_rom, Ti_rom


def gusts():
    T_rom = sio.loadmat('model/reduced/T_rom.mat')["T_rom"]
    H5gust = sio.loadmat('model/discretegusts/H5gust.mat')["arr"]
    H5gust = np.matmul(T_rom, H5gust)
    
    H10gust = sio.loadmat('model/discretegusts/H10gust.mat')["arr"]
    H10gust = np.matmul(T_rom, H10gust)

    H15gust = sio.loadmat('model/discretegusts/H15gust.mat')["arr"]
    H15gust = np.matmul(T_rom, H15gust)
    
    H20gust = sio.loadmat('model/discretegusts/H20gust.mat')["arr"]
    H20gust = np.matmul(T_rom, H20gust)

    return H5gust, H10gust, H15gust, H20gust