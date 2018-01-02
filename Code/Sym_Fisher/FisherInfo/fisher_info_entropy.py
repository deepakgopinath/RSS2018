from sympy import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


from IPython import embed

class FisherInfo(object):
    """docstring forFisherInfo."""
    def __init__(self):
        super(FisherInfo, self).__init__()
        self.arg = arg
        self.ng = 3 #number of goals.
        self.nd = 2 #dimensionality of the problem
        self.num_modes = 3 #number of control modes
        self.sig_t = 0.1 # standard deviation for the measurement model

        self.xg_t =  np.random.choice(np.arange(-3,3), size=(self.nd, self.ng)) + np.random.rand(self.nd, 1) - np.random.rand(self.nd, 1)
        # self.xg_t = np.array([[-3, 3, 2], [3, 3, 4]]); # hardcoded goal position for ng = 3
        self.xr_t =  np.random.choice(np.arange(-3,3), size=(self.nd, 1)) + np.random.rand(self.nd, 1) - np.random.rand(self.nd, 1)
        self.xr_t = np.array([[1], [0]]); #hardcoded robot position
        self.uh_t = np.array([[1], [0]]); #hardcoded control command

        #Plot the goals.
        fig, ax = plt.subplots()
        ax.scatter(self.xg_t[0, :], self.xg_t[1, :], s=100)
        ax.hold(True)
        ax.scatter(self.xr_t[0,0], self.xr_t[1,0], s = 100, c='r')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Goal configuration')
        ax.grid(True)
        plt.show()

        # ax.scatter()
        embed()

        #Symbolic variables. Vector of variables for 3D robot position and 3D control command.

        self.xr = MatrixSymbol('xr', self.nd, 1)
        self.uh = MatrixSymbol('uh', self.nd, 1)
        self.sig = Symbol('sig')
        self.normuh = self.uh



    def compute_best_mode(self):
        pass

if __name__ == '__main__':
    FI = FisherInfo()
    # FI.compute()
