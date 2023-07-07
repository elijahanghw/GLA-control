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

        ## Sim parameters
        num_chord = 8
        ref_chord = 0.2743
        V_inf = 10
        ds = 1/num_chord
        self.dt = ds*ref_chord/V_inf

        # Deflection action
        self.action_space = np.array([[-20/180*np.pi, 20/180*np.pi], [-20/180*np.pi, 20/180*np.pi], [-20/180*np.pi, 20/180*np.pi]])

        # Rate action
        # self.action_space = np.array([[-50/180*np.pi, 50/180*np.pi], [-50/180*np.pi, 50/180*np.pi], [-50/180*np.pi, 50/180*np.pi]])

        self.norm_action_space = np.array([[-1,1], [-1,1], [-1,1]])

        self.state_space = np.array([[-5, 5], [-1, 1], [-1, 1]])
        # self.state_space = np.zeros((18,2))

    def reset(self):
        self.t = 0
        self.x_old = np.zeros(self.A.shape[0])
        self.U = np.zeros(self.action_space.shape[0])
        self.U_dot = np.zeros(self.action_space.shape[0])
        # Initialise run with randomly selected gust
        self.gust = random.choice(self.gust_list)

        return np.matmul(self.C, self.x_old)
    
    def select_gust(self, selection):
        if selection == "H5":
            self.gust = self.H5gust
        elif selection == "H10":
            self.gust = self.H10gust
        elif selection == "H15":
            self.gust = self.H15gust
        elif selection == "H20":
            self.gust = self.H20gust
        else:
            self.gust = self.H5gust

    
    def step(self, action):
        # Deflection action
        self.U = self.inv_norm_act(action)
        
        # Rate action
        # self.U_dot = self.inv_norm_act(action)
        # self.U = self.U + self.U_dot*self.dt

        x_new = np.matmul(self.A, self.x_old) + np.matmul(self.B, self.U) + self.gust[:,self.t]
        y_new = np.matmul(self.C, x_new)
        y_old = np.matmul(self.C, self.x_old)
        plunge = y_new[0]
        pitch = y_new[1]
        bend = y_new[2]
        
        new_state = np.array([plunge, 180/np.pi*pitch, 200*bend])
        # reward = -10*sum(abs(x_new))
        reward = (-abs(plunge) - 180/np.pi*abs(pitch) - 10000*abs(bend))
        # reward = 0.01*(1/(abs(plunge)+0.01) + 1/(180/np.pi*abs(pitch)+0.01) + 1/((1000*abs(bend))+0.01) - 3)
        # reward = -(abs(y_new[0]) - abs(y_old[0]) +  10*180/np.pi*(abs(y_new[1]) - abs(y_old[1])) + 1000*(abs(y_new[0]) - abs(y_old[0])))/0.05

        self.x_old = x_new
        self.t += 1
        
        return new_state, reward
    
    def inv_norm_act(self, action):
        m = (self.action_space[:,1] - self.action_space[:,0])/2
        c = (self.action_space[:,1] + self.action_space[:,0])/2
        return m*action + c

    def norm_act(self, action):
        m = 2/(self.action_space[:,1] - self.action_space[:,0])
        b = (self.action_space[:,1] + self.action_space[:,0])/2
        return m *(action - b)
        


