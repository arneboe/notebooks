from math import log
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

class CS:
    def __init__(self, executionTime, lastPhaseValue = 0.01):
        self.T = executionTime
        self.alpha = -log(lastPhaseValue)
        self.z = 1.0 #current phase value
        self.t = 0.0#current time

    def step(self, dt):
        '''
        initially the cs is at phase 1. the first call to step will move it.
        '''
        self.z += (-self.alpha * self.z / self.T) * dt
        self.t += dt
        return self.z

    def get_phases(self, times):
        return np.exp(-self.alpha / self.T * times)

    def reset(self):
        self.z = 1.0
        self.t = 0.0


if __name__ == "__main__":
    executionTime = 1.5
    dt = 0.01
    cs = CS(executionTime)
    numPhases = executionTime / dt
    numSteps = int(numPhases + 1)
    times = []
    values = []

    t = 0.0
    while(t < executionTime):
        times.append(t)
        values.append(cs.z)
        cs.step(dt)
        t += dt
    #get value from last step
    times.append(t)
    values.append(cs.z)
    plt.plot(times, values)
    
    times = []
    values = []
    executionTime = 1
    dt = 0.01
    cs = CS(executionTime)
    numPhases = executionTime / dt
    numSteps = int(numPhases + 1)
    times = []
    values = []

    t = 0.0
    while(t < executionTime):
        times.append(t)
        values.append(cs.z)
        cs.step(dt)
        t += dt
    times.append(t)
    values.append(cs.z)
    plt.plot(times, values)
    times = []
    values = []
    executionTime = 0.5
    dt = 0.01
    cs = CS(executionTime)
    numPhases = executionTime / dt
    numSteps = int(numPhases + 1)
    times = []
    values = []

    t = 0.0
    while(t < executionTime):
        times.append(t)
        values.append(cs.z)
        cs.step(dt)
        t += dt
    times.append(t)
    values.append(cs.z)
    plt.plot(times, values)
    plt.xlabel("Zeit in Sekunden", fontsize=22)
    plt.ylabel("Phasenwert", fontsize=22)
    #plt.title("Drei verschiedene Canonical Systems")
    
    plt.savefig("cs.pdf")

