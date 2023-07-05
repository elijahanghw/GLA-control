import scipy.io as sio
import random
import numpy as np

class Environment:
    def __init__(self):
        self.A = sio.loadmat('model/reduced/A_rsys.mat')["A_rsys"]
        self.B = sio.loadmat('model/reduced/B_rsys.mat')["B_rsys"]
        self.C = sio.loadmat('model/reduced/C_rsys.mat')["C_rsys"]
        self.D = sio.loadmat('model/reduced/D_rsys.mat')["D_rsys"]

        self.T_rom = sio.loadmat('model/reduced/T_rom.mat')["T_rom"]
        self.Ti_rom = sio.loadmat('model/reduced/Ti_rom.mat')["Ti_rom"]

        self.H5gust = sio.loadmat('model/discretegusts/H5gust.mat')["arr"]
        self.H5gust = np.matmul(self.T_rom, self.H5gust)
        self.H10gust = sio.loadmat('model/discretegusts/H10gust.mat')["arr"]
        self.H10gust = np.matmul(self.T_rom, self.H10gust)
        self.H15gust = sio.loadmat('model/discretegusts/H15gust.mat')["arr"]
        self.H15gust = np.matmul(self.T_rom, self.H15gust)
        self.H20gust = sio.loadmat('model/discretegusts/H20gust.mat')["arr"]
        self.H20gust = np.matmul(self.T_rom, self.H20gust)

        self.gust_list = [self.H5gust, self.H10gust, self.H15gust, self.H20gust]

        # Deflection action
        self.action_space = np.array([[-20/180*np.pi, 20/180*np.pi], [-20/180*np.pi, 20/180*np.pi], [-20/180*np.pi, 20/180*np.pi]])
        self.norm_action_space = np.array([[-1,1], [-1,1], [-1,1]])

        # Rate action
        # self.action_space = np.array([[-50/180*np.pi, 50/180*np.pi], [-50/180*np.pi, 50/180*np.pi], [-50/180*np.pi, 50/180*np.pi]])

        self.state_space = np.array([[-5, 5], [-1, 1], [-1, 1]])

    def reset(self):
        self.t = 0
        self.x_old = np.zeros(self.A.shape[0])
        self.U = np.zeros(self.action_space.shape[0])
        # Initialise run with randomly selected gust
        self.gust = random.choice(self.gust_list)

        return np.matmul(self.C, self.x_old)
    
    def step(self, action):
        # Deflection action
        self.U = self.inv_norm_act(action)

        # Rate action
        # self.U_dot = self.inv_norm_act(action)
        # self.U += self.U_dot

        x_new = np.matmul(self.A, self.x_old) + np.matmul(self.B, self.U) + self.gust[:,self.t]
        y = np.matmul(self.C, x_new)
        plunge = y[0]
        pitch = y[1]
        bend = y[2]
        self.x_old = x_new
        self.t += 1

        new_state = np.array([plunge, pitch, bend])
        reward = 0.003*(-abs(plunge) - 180/np.pi*abs(pitch) - 200*abs(bend))
        
        return new_state, reward
    
    def inv_norm_act(self, action):
        m = (self.action_space[:,1] - self.action_space[:,0])/2
        c = (self.action_space[:,1] + self.action_space[:,0])/2
        return m*action + c

    def norm_act(self, action):
        m = 2/(self.action_space[:,1] - self.action_space[:,0])
        b = (self.action_space[:,1] + self.action_space[:,0])/2
        return m *(action - b)
        


