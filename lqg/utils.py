import numpy as np

def log10uniform(low=0, high=1, size=None):
    return np.power(10 , np.random.uniform(low, high, size))

def ga_lqr():
    pass