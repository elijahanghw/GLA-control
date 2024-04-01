<<<<<<< HEAD
import numpy as np
import scipy.io as sio
class Controller:
    def __init__(self, individual=None):
        self.individual = individual

    def setup(self):
        self.weights1 = self.individual[0:24].reshape((3,8))
        # self.bias1 = self.individual[12:16].reshape((1,4))
        self.weights2 = self.individual[24:88].reshape((8,8))
        # self.bias2 = self.individual[32:36].reshape((1,4))
        self.weights3 = self.individual[88:112].reshape((8,3))
        # self.bias3 = self.individual[48:51].reshape((1,3))

    def forward(self, inputs):
        inputs = inputs.reshape((1,3))
        layer1 = np.matmul(inputs, self.weights1) # + self.bias1
        layer1 = np.tanh(layer1)

        layer2 = np.matmul(layer1, self.weights2) # + self.bias2
        layer2 = np.tanh(layer2)

        output = np.matmul(layer2, self.weights3) # + self.bias3
        output = 20*np.pi/180*np.tanh(output)
        
        return output[0]
    
    def save(self, path):
        sio.savemat(path, mdict={"weights1": self.weights1, "weights2": self.weights2, "weights3": self.weights3})

    def load(self, path):
        self.weights1 = sio.loadmat(path)["weights1"]
        # self.bias1 = sio.loadmat(path)["bias1"]
        self.weights2 = sio.loadmat(path)["weights2"]
        # self.bias2 = sio.loadmat(path)["bias2"]
        self.weights3 = sio.loadmat(path)["weights3"]
=======
import numpy as np
import scipy.io as sio
class Controller:
    def __init__(self, individual=None):
        self.individual = individual

    def setup(self):
        self.weights1 = self.individual[0:24].reshape((3,8))
        # self.bias1 = self.individual[12:16].reshape((1,4))
        self.weights2 = self.individual[24:88].reshape((8,8))
        # self.bias2 = self.individual[32:36].reshape((1,4))
        self.weights3 = self.individual[88:112].reshape((8,3))
        # self.bias3 = self.individual[48:51].reshape((1,3))

    def forward(self, inputs):
        inputs = inputs.reshape((1,3))
        layer1 = np.matmul(inputs, self.weights1) # + self.bias1
        layer1 = np.tanh(layer1)

        layer2 = np.matmul(layer1, self.weights2) # + self.bias2
        layer2 = np.tanh(layer2)

        output = np.matmul(layer2, self.weights3) # + self.bias3
        output = 20*np.pi/180*np.tanh(output)
        
        return output[0]
    
    def jacobian(self):
        W1 = np.transpose(self.weights1)
        W2 = np.transpose(self.weights2)
        W3 = np.transpose(self.weights3)

        J1 = np.matmul(20*np.pi/180*np.eye(3), W3)
        J2 = np.matmul(J1, W2)
        J3 = np.matmul(J2, W1)
        return J3
    
    def save(self, path):
        sio.savemat(path, mdict={"weights1": self.weights1, "weights2": self.weights2, "weights3": self.weights3})

    def load(self, path):
        self.weights1 = sio.loadmat(path)["weights1"]
        # self.bias1 = sio.loadmat(path)["bias1"]
        self.weights2 = sio.loadmat(path)["weights2"]
        # self.bias2 = sio.loadmat(path)["bias2"]
        self.weights3 = sio.loadmat(path)["weights3"]
>>>>>>> c339e5e208710fac150003b1e3154d27225865e9
        # self.bias3 = sio.loadmat(path)["bias3"]