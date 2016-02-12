from cs import CS
from matplotlib import pyplot as plt


class SimpleDmp:
    '''
    A simple Ijspeert dmp without forcing term
    '''
    def __init__(self, executionTime, startPos, startVel, goalPos):
        self.T = executionTime
        self.alpha = 25.0
        self.beta = 6.25
        self.g = goalPos
        self.y = startPos
        self.z = self.T * startVel;

    def step(self, dt):
        f = 0.0 #fimxe introduce forcing term
        zd = ((self.alpha * (self.beta * (self.g - self.y)- self.z) + f) / self.T) * dt
        yd = self.z / self.T * dt
        self.y += yd
        self.z += zd

    def run(self, dt, startT, endT):
        '''
        runs the whole dmp and returns ([ts], [ys], [yds])
        '''
        ts = []
        ys = []
        yds = []
        t = startT
        while t < endT:
            ts.append(t)
            ys.append(self.y)
            yds.append(self.z / self.T)
            t += dt
            self.step(dt)
        ts.append(t)
        ys.append(self.y)
        yds.append(self.z / self.T)

        return (ts, ys, yds)


if __name__ == "__main__":
    executionTime = 1.5
    startPos = 0
    startVel = 0
    endPos = 10
    dt = 0.01
    dmp = SimpleDmp(executionTime, startPos, startVel, endPos)
    (ts, ys, yds) = dmp.run(dt, 0.0, executionTime)
    plt.plot(ts, ys, "b")

    plt.ylabel("Motorposition in Grad", fontsize=20)
    plt.xlabel("Zeit in Sekunden", fontsize=20)

    #change goal pos a few times
    dmp = SimpleDmp(executionTime, startPos, startVel, endPos)
    (ts, ys, yds) = dmp.run(dt, 0.0, 0.2)
    plt.plot(ts, ys, "b")
    dmp.g = 5
    (ts, ys, yds) = dmp.run(dt, 0.2, 0.4)
    plt.plot(ts, ys, "g")
    dmp.g = 15
    (ts, ys, yds) = dmp.run(dt, 0.4, 0.7)
    plt.plot(ts, ys, "r")
    dmp.g = 10
    (ts, ys, yds) = dmp.run(dt, 0.7, executionTime)
    plt.plot(ts, ys, "y")
    plt.savefig("dmp_change_goal_without_f.pdf")
    
    #change executionTime a few times
    #dmp = SimpleDmp(executionTime, startPos, startVel, endPos)
    #plt.ylim(-0.1, 14.0)
    #(ts, ys, yds) = dmp.run(dt, 0.0, 0.2)
    #plt.plot(ts, ys, "b")
    #dmp.T = 4
    #(ts, ys, yds) = dmp.run(dt, 0.2, 0.4)
    #plt.plot(ts, ys, "g")
    #dmp.T = 0.5
    #(ts, ys, yds) = dmp.run(dt, 0.4, 0.7)
    #plt.plot(ts, ys, "r")
    #dmp.T = 1.5
    #(ts, ys, yds) = dmp.run(dt, 0.7, executionTime)
    #plt.plot(ts, ys, "y")
    #plt.savefig("dmp_change_time_without_f.pdf")
    #plt.show()
