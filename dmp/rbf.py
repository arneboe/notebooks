from matplotlib import pyplot as plt
from cs import CS
import numpy as np
from math import log

class Rbf:
    '''
    A simple radial basis function approximator
    '''
    def __init__(self, cs, executionTime, numWeights = 30, overlap = 0.1):
        '''
        The cs is only needed to spread the centers.
        ExecutionTime is only needed to spread the centers. needs to be the original executionTime the the weights have been learned with
        '''
        self.numWeights = numWeights
        self.cs = cs
        self.executionTime = executionTime
        #evenly spread the centers throughout the execution time
        centers_in_time = np.linspace(start=0.0, stop=executionTime, num=numWeights)
        #and move them to phase space
        self.centers = np.exp(-cs.alpha / executionTime * centers_in_time)
        self.weights = np.zeros_like(self.centers)

        #set the widths of each rbf according to overlap
        self.widths = np.ndarray(numWeights)
        log_overlap = -log(overlap)
        for i in range(1, numWeights):
            self.widths[i - 1] = log_overlap / ((self.centers[i] - self.centers[i - 1])**2)
        self.widths[numWeights - 1] = self.widths[numWeights - 2];


    def set_weights(self, weights):
        assert(len(weights) == self.numWeights)
        self.weights = weights

    def evaluate(self, z):
        psi = np.exp(-self.widths * (z - self.centers)**2)
        nom = np.dot(self.weights, psi) * z
        denom = np.sum(psi)
        return nom / denom

    def plot_gaussians(self):
        points_in_time = np.linspace(0.0, executionTime * 1.3, num=600)
        points_in_phase = self.cs.get_phases(points_in_time)
        for i in range(self.numWeights):#for each guassian
            values = np.exp(-self.widths[i] * (points_in_phase - self.centers[i])**2)
            plt.plot(points_in_phase, values)
            plt.plot(points_in_time, values)

    def plot_function(self):
        points_in_time = np.linspace(0.0, executionTime * 1.3, num=600)
        points_in_phase = self.cs.get_phases(points_in_time)
        values = []
        for z in points_in_phase:
            values.append(self.evaluate(z))
        plt.plot(points_in_time, values)

    def psi(self, i, phases):
        '''
        evaluates the i'th gaussian at the specified phases and returns the results as vector
        '''
        assert(i < self.numWeights)
        assert(i >= 0)
        return np.exp(-self.widths[i] * (phases - self.centers[i])**2)


if __name__ == "__main__":
    executionTime = 5
    cs = CS(executionTime)
    rbf = Rbf(cs, executionTime)
    rbf.set_weights(np.random.rand(rbf.numWeights))
    rbf.plot_function()
    plt.show()