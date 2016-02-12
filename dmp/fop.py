import numpy as np
from matplotlib import pyplot as plt

class Fop:
    def __init__(self, x0, fx0, fx0d, fx0dd, x1, fx1, fx1d, fx1dd):
        x02 = x0 * x0
        x03 = x02 * x0
        x04 = x03 * x0
        x05 = x04 * x0
        x12 = x1 * x1
        x13 = x12 * x1
        x14 = x13 * x1
        x15 = x14 * x1

        A = np.asarray([[1, x0, x02   , x03    , x04     , x05],
                       [0, 1 , 2 * x0, 3 * x02, 4 * x03 , 5 * x04],
                       [0, 0 , 2     , 6 * x0 , 12 * x02, 20 * x03],
                       [1, x1, x12   , x13    , x14     , x15],
                       [0, 1 , 2 * x1, 3 * x12, 4 * x13 , 5 * x14],
                       [0, 0 , 2     , 6 * x1 , 12 * x12, 20 * x13]])

        b = np.asarray([fx0, fx0d, fx0dd, fx1, fx1d, fx1dd])
        self.c = np.linalg.solve(A, b)
        
    def eval(self, x):
        '''
        @returns [f(x), f'(x), f''(x)]^T
        '''
        x2 = x * x
        x3 = x2 * x
        x4 = x3 * x
        x5 = x4 * x

        result = np.ndarray(3)
        result[0] = self.c[0] + self.c[1] * x + self.c[2] * x2 + self.c[3] * x3 + self.c[4] * x4 + self.c[5] * x5
        result[1] = self.c[1] + 2 * self.c[2] * x + 3 * self.c[3] * x2 + 4 * self.c[4] * x3 + 5 * self.c[5] * x4
        result[2] = 2 * self.c[2] + 6 * self.c[3] * x + 12 * self.c[4] * x2 + 20 * self.c[5] * x3

        return result

    def evalArray(self, xs):
        '''
        returns (positions, velocities, aceelerations)
        '''
        pos = np.ndarray(len(xs))
        vel = np.ndarray(len(xs))
        acc = np.ndarray(len(xs))
        for i, x in enumerate(xs):
            y, yd, ydd = self.eval(x)
            pos[i] = y
            vel[i] = yd
            acc[i] = ydd
        return (pos, vel, acc)



if __name__ == "__main__":
    fop = Fop(0, 1, 2, 3, 5, 42, 24, 11)
    xs = np.linspace(-0.3, 5.3, 200)
    pos, vel, acc = fop.evalArray(xs)
    plt.plot(xs, pos, "r")
    plt.plot(xs, vel, "g")
    plt.plot(xs, acc, "b")
    plt.legend(["Position", "Velocity", "Acceleration"])
    plt. show()
