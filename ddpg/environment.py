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

        self.gust1 = sio.loadmat('model/contgust/contgust1.mat')["gustz"]
        self.gust2 = sio.loadmat('model/contgust/contgust2.mat')["gustz"]
        self.gust3 = sio.loadmat('model/contgust/contgust3.mat')["gustz"]
        
        self.gusta = np.zeros((2528, self.gust1.shape[0]))
        self.gustb = np.zeros((2528, self.gust2.shape[0]))
        self.gustc = np.zeros((2528, self.gust3.shape[0]))

        for i, _ in enumerate(self.gust1):
            self.gusta[0:208,i] = 0.006*self.gust1[i]
            self.gustb[0:208,i] = 0.006*self.gust2[i]
            self.gustc[0:208,i] = 0.006*self.gust3[i]

        self.gusta = np.matmul(self.T_rom, self.gusta)
        self.gustb = np.matmul(self.T_rom, self.gustb)
        self.gustc = np.matmul(self.T_rom, self.gustc)

        self.nogust = np.zeros_like(self.H5gust)

        self.gust_list = [self.H5gust, self.H10gust, self.H15gust, self.H20gust, self.nogust]
        self.gust_list_cont = [self.gusta, self.gustb, self.gustc, self.nogust]

        ## Sim parameters
        num_chord = 8
        ref_chord = 0.2743
        V_inf = 10
        ds = 1/num_chord
        self.dt = ds*ref_chord/V_inf

        # Deflection action
        # self.action_space = np.array([[-20/180*np.pi, 20/180*np.pi], [-20/180*np.pi, 20/180*np.pi], [-20/180*np.pi, 20/180*np.pi]])

        # Rate action
        self.action_space = np.array([[-100/180*np.pi, 100/180*np.pi], [-100/180*np.pi, 100/180*np.pi], [-100/180*np.pi, 100/180*np.pi]])

        self.norm_action_space = np.array([[-1,1], [-1,1], [-1,1]])

        self.state_space = np.zeros([3,2])
        # self.state_space = np.zeros((18,2))

    def reset(self):
        self.t = 0
        self.x_old = np.zeros(self.A.shape[0])
        self.U = np.zeros(self.action_space.shape[0])
        self.U_dot = np.zeros(self.action_space.shape[0])
        # Initialise run with randomly selected gust
        self.gust = random.choice(self.gust_list_cont)

        return np.array([0.0, 0.0, 0.0])
    
    
    def select_gust(self, selection):
        if selection == "H5":
            self.gust = self.H5gust
        elif selection == "H10":
            self.gust = self.H10gust
        elif selection == "H15":
            self.gust = self.H15gust
        elif selection == "H20":
            self.gust = self.H20gust
        elif selection == "A":
            self.gust = self.gusta
        elif selection == "B":
            self.gust = self.gustb
        elif selection == "C":
            self.gust = self.gustc
        else:
            self.gust = self.nogust

    
    def step(self, action):
        # Deflection action
        # self.U = self.inv_norm_act(action)
        
        # Rate action
        self.U_dot = self.inv_norm_act(action)
        U_new = np.clip(self.U + self.U_dot*self.dt, -20*np.pi/180, 20*np.pi/180)

        x_new = np.matmul(self.A, self.x_old) + np.matmul(self.B, U_new) + self.gust[:,self.t]
        y_new = np.matmul(self.C, x_new)
        y_old = np.matmul(self.C, self.x_old)
        plunge_new = y_new[0]
        pitch_new = y_new[1]
        bend_new = y_new[2]

        plunge_old = y_old[0]
        pitch_old = y_old[1]
        bend_old = y_old[2]
        
        new_state = np.array([plunge_new, 180/np.pi*pitch_new, 200*bend_new])
        # reward = -10*sum(abs(x_new))
        reward = -0.001*(abs(plunge_new)**2 + (180/np.pi*abs(pitch_new))**2 + (2000*abs(bend_new))**2)
        # reward = 0.01*(1/(abs(plunge)+0.01) + 1/(180/np.pi*abs(pitch)+0.01) + 1/((1000*abs(bend))+0.01) - 3)
        # reward = -(abs(y_new[0]) - abs(y_old[0]) +  10*180/np.pi*(abs(y_new[1]) - abs(y_old[1])) + 10000*(abs(y_new[0]) - abs(y_old[0])))/0.05

        self.x_old = x_new
        self.U = U_new
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
        


