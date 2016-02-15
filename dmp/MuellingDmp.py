from cs import CS
from rbf import Rbf
import numpy as np
from math import exp
from fop import Fop

class MuellingDmp:
    '''
    Muelling dmp without amplitude scaling (eta)
    '''
    def __init__(self, executionTime, startPos, startVel, startAcc, goalPos, goalVel, cs, numWeights, overlap, use_vel_scaling):
        self.T = executionTime
        self.cs = cs
        self.alpha = 25.0
        self.beta = 6.25
        self.g = goalPos
        self.gd = goalVel
        self.gdd = 0.0 #has to be 0
        self.y = startPos
        self.y0 = startPos
        self.yd0 = startVel
        self.ydd0 = startAcc
        self.ydd = startAcc
        self.v = self.T * self.yd0
        self.startV = self.v
        self.fop = Fop(0.0, startPos, startVel, startAcc, executionTime, goalPos, goalVel, self.gdd)
        self.rbf = Rbf(cs, executionTime, numWeights, overlap)
        self.amplitude = goalPos - startPos
        self.amplitude2 = goalVel - startVel
        self.use_vel_scaling = use_vel_scaling


    def step(self, dt):
        z = self.cs.step(dt)
        t = self.cs.t
        f = self.rbf.evaluate(z)
       # eta = (self.g - self.y0) / self.amplitude
        #eta = exp((self.g - self.y0) - self.amplitude)
        eta = (self.gd - self.yd0) / self.amplitude2
        if self.use_vel_scaling:
            f *= eta
        g, gd, gdd = self.fop.eval(t) #moving target
        vd = ((self.alpha * (self.beta * (g - self.y) + gd * self.T - self.v) + gdd * self.T**2 + f) / self.T) * dt
        yd = self.v / self.T * dt
        self.y += yd
        self.v += vd
        self.ydd = vd / self.T

    def imitate(self, times, positions, velocities, accelerations,  dt):
        '''
        first position at t=0 and last position at t = executionTime
        dt = sampling dt
        '''
        fop = Fop(0.0, positions[0], velocities[0], accelerations[0], times[-1], positions[-1], velocities[-1], accelerations[-1])
        g, gd, gdd = fop.evalArray(times)
        references = self.T**2 * accelerations - self.alpha * ( self.beta * (g - positions) + self.T * gd - self.T * velocities) - self.T**2 * gdd
        phases = self.cs.get_phases(times)
        weights = np.ndarray(self.rbf.numWeights)
        for i in range(self.rbf.numWeights):
            psi = self.rbf.psi(i, phases)
            psiD = np.diag(psi)
            weights[i] = np.linalg.inv([[np.dot(phases.T, np.dot(psiD, phases))]]) * np.dot(phases.T, np.dot(psiD, references))
        self.rbf.set_weights(weights)
        self.amplitude = positions[-1] - positions[0]


    def reset(self, cs, g, T, y0, gd):
        self.cs = cs
        self.g = g
        self.gd = gd
        self.T = T
        self.y = y0
        self.v = self.startV
        self.y0 = y0
        self.fop = Fop(0.0, self.y0, self.yd0, self.ydd0, self.T, self.g, self.gd, self.gdd)


    def run(self, dt):
        '''
        runs the whole dmp and returns ([ts], [ys], [yds])
        '''
        ts = []
        ys = []
        yds = []
        ydds = []
        t = 0.0
        while t < self.T:
            ts.append(t)
            ys.append(self.y)
            yds.append(self.v / self.T)
            ydds.append(self.ydd)
            t += dt
            self.step(dt)
        ts.append(t)
        ys.append(self.y)
        yds.append(self.v / self.T)
        ydds.append(self.ydd)
        return (ts, np.asarray(ys), np.asarray(yds), np.asarray(ydds))