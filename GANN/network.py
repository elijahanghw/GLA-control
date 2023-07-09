import numpy as np
import scipy.io as sio
class Controller:
    def __init__(self, individual=None):
        self.individual = individual

    def setup(self):
        self.weights1 = self.individual[0:12].reshape((3,4))
        # self.bias1 = self.individual[12:16].reshape((1,4))
        self.weights2 = self.individual[12:28].reshape((4,4))
        # self.bias2 = self.individual[32:36].reshape((1,4))
        self.weights3 = self.individual[28:40].reshape((4,3))
        # self.bias3 = self.individual[48:51].reshape((1,3))

    def forward(self, inputs):
        inputs = inputs.reshape((1,3))
        layer1 = np.matmul(inputs, self.weights1) # + self.bias1
        layer1 = np.maximum(0, layer1)

        layer2 = np.matmul(layer1, self.weights2) # + self.bias2
        layer2 = np.maximum(0, layer2)

        output = np.matmul(layer2, self.weights3) # + self.bias3
        output = 20*np.pi/180*np.tanh(0.05*output)
        
        return output[0]
    
    def save(self, path):
        sio.savemat(path, mdict={"weights1": self.weights1, "weights2": self.weights2, "weights3": self.weights3})

    def load(self, path):
        self.weights1 = sio.loadmat(path)["weights1"]
        # self.bias1 = sio.loadmat(path)["bias1"]
        self.weights2 = sio.loadmat(path)["weights2"]
        # self.bias2 = sio.loadmat(path)["bias2"]
        self.weights3 = sio.loadmat(path)["weights3"]
        # self.bias3 = sio.loadmat(path)["bias3"]