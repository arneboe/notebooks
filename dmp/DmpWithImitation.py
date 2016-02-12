from rbf import Rbf
import numpy as np
class DmpWithImitation:
    '''
    A simple Ijspeert dmp with forcing term
    '''
    def __init__(self, executionTime, startPos, startVel, goalPos, cs, numWeights, overlap, use_scaling):
        self.T = executionTime
        self.cs = cs
        self.alpha = 25.0
        self.beta = 6.25
        self.g = goalPos
        self.y = startPos
        self.startPos = startPos
        self.z = self.T * startVel;
        self.startZ = self.z
        self.rbf = Rbf(cs, executionTime, numWeights, overlap)
        self.amplitude = 0
        self.use_scaling = use_scaling

    def step(self, dt):
        z = self.cs.step(dt)
        f = self.rbf.evaluate(z)
        if self.use_scaling:
            f *= (self.g - self.startPos) / self.amplitude
            
        zd = ((self.alpha * (self.beta * (self.g - self.y)- self.z) + f) / self.T) * dt
        yd = self.z / self.T * dt
        self.y += yd
        self.z += zd

    def imitate(self, times, positions, dt):
        '''
        first position at t=0 and last position at t = executionTime
        dt = sampling dt
        '''
        self.amplitude = positions[-1] - positions[0]
        velocities = np.gradient(positions, dt)
        accelerations = np.gradient(velocities, dt)
        goal = positions[len(positions) - 1]
        references = self.T**2 * accelerations - self.alpha * (self.beta * (goal - positions) - self.T * velocities)
        phases = self.cs.get_phases(times)
        weights = np.ndarray(self.rbf.numWeights)
        for i in range(self.rbf.numWeights):
            psi = self.rbf.psi(i, phases)
            psiD = np.diag(psi)
            weights[i] = np.linalg.inv([[np.dot(phases.T, np.dot(psiD, phases))]]) * np.dot(phases.T, np.dot(psiD, references))
        self.rbf.set_weights(weights)


    def run(self, dt, startTime = 0.0, endTime = None):
        '''
        runs the whole dmp and returns ([ts], [ys], [yds])
        '''
        ts = []
        ys = []
        yds = []
        t = startTime
        if endTime is None:
            endTime = self.T
        while t < endTime:
            ts.append(t)
            ys.append(self.y)
            yds.append(self.z / self.T)
            t += dt
            self.step(dt)
        #ts.append(t)
        #ys.append(self.y)
        #yds.append(self.z / self.T)
        return (ts, ys, yds)

    def reset(self, cs, goal, executionTime, start):
        self.cs = cs
        self.g = goal
        self.T = executionTime
        self.y = start
        self.z = self.startZ
