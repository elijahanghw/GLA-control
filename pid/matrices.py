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
    H1gust = sio.loadmat('model/discretegusts/H1gust.mat')["arr"]
    H1gust = np.matmul(T_rom, H1gust)
    
    H2gust = sio.loadmat('model/discretegusts/H2gust.mat')["arr"]
    H2gust = np.matmul(T_rom, H2gust)

    H3gust = sio.loadmat('model/discretegusts/H3gust.mat')["arr"]
    H3gust = np.matmul(T_rom, H3gust)
    
    H5gust = sio.loadmat('model/discretegusts/H5gust.mat')["arr"]
    H5gust = np.matmul(T_rom, H5gust)

    return H1gust, H2gust, H3gust, H5gust