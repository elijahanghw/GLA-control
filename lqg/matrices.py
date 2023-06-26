import scipy.io as sio
import numpy as np

def system_mat():
    A = sio.loadmat('lqg\A_rsys.mat')["A_rsys"]
    B = sio.loadmat('lqg\B_rsys.mat')["B_rsys"]
    C = sio.loadmat('lqg\C_rsys.mat')["C_rsys"]
    D = sio.loadmat('lqg\D_rsys.mat')["D_rsys"]

    T_rom = sio.loadmat('lqg\T_rom.mat')["T_rom"]
    Ti_rom = sio.loadmat('lqg\Ti_rom.mat')["Ti_rom"]

    return A, B, C, D, T_rom, Ti_rom


def gusts():
    T_rom = sio.loadmat('lqg\T_rom.mat')["T_rom"]
    H1gust = sio.loadmat('lqg\discretegusts\H1gust.mat')["arr"]
    H1gust = np.matmul(T_rom, H1gust)
    
    H2gust = sio.loadmat('lqg\discretegusts\H2gust.mat')["arr"]
    H2gust = np.matmul(T_rom, H2gust)

    H3gust = sio.loadmat('lqg\discretegusts\H3gust.mat')["arr"]
    H3gust = np.matmul(T_rom, H3gust)
    
    H5gust = sio.loadmat('lqg\discretegusts\H5gust.mat')["arr"]
    H5gust = np.matmul(T_rom, H5gust)

    return H1gust, H2gust, H3gust, H5gust