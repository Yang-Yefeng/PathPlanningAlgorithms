import numpy as np
import pandas as pd


class uav:
    def __init__(self, initP, initV, initA, initJ, initS, maxV, maxA, maxJ, maxS, minV, minA, minJ, minS, flag: int):
        """
        :brief:             不说也知道是啥意思，所以不写了，嫌麻烦......^_^
        :param initP:
        :param initV:
        :param initA:
        :param initJ:
        :param initS:
        :param maxV:
        :param maxA:
        :param maxJ:
        :param maxS:
        :param minV:
        :param minA:
        :param minJ:
        :param minS:
        """
        self.initP, self.initV, self.initA, self.initJ, self.initS = initP, initV, initA, initJ, initS
        self.maxV, self.maxA, self.maxJ, self.maxS = maxV, maxA, maxJ, maxS
        self.minV, self.minA, self.minJ, self.minS = minV, minA, minJ, minS
        self.P, self.V, self.A, self.J, self.S = self.initP, self.initV, self.initA, self.initJ, self.initS
        self.dt = 0.01      # 10ms
        self.time = 0.0
        self.flag = flag    # 0-acc, 1-jerk, 2-snap

        '''data save'''
        self.Px = [self.initP[0]]
        self.Py = [self.initP[1]]
        self.Pz = [self.initP[2]]

        self.Vx = [self.initV[0]]
        self.Vy = [self.initV[1]]
        self.Vz = [self.initV[2]]

        self.Ax = [self.initA[0]]
        self.Ay = [self.initA[1]]
        self.Az = [self.initA[2]]

        self.T = [self.time]

        if self.initJ is not None:
            self.Jx = [self.initJ[0]]
            self.Jy = [self.initJ[1]]
            self.Jz = [self.initJ[2]]
        else:
            self.Jx = None
            self.Jy = None
            self.Jz = None

        if self.initS is not None:
            self.Sx = [self.initS[0]]
            self.Sy = [self.initS[1]]
            self.Sz = [self.initS[2]]
        else:
            self.Sx = None
            self.Sy = None
            self.Sz = None
        '''data save'''

    def state_saturation_check(self):
        for i in range(3):
            self.V[i] = np.max([np.min([self.V[i], self.maxV[i]]), self.minV[i]])
            self.A[i] = np.max([np.min([self.A[i], self.maxA[i]]), self.minA[i]])
        if self.flag > 0:       # 至少有jerk
            for i in range(3):
                self.J[i] = np.max([np.min([self.J[i], self.maxJ[i]]), self.minJ[i]])
            if self.flag > 1:
                for i in range(3):
                    self.S[i] = np.max([np.min([self.S[i], self.maxS[i]]), self.minS[i]])

    def input_saturation_check(self, _input):
        if self.flag == 0:
            _input = [np.max([np.min([_input[i], self.maxA[i]]), self.minA[i]]) for i in range(3)]
        elif self.flag == 1:
            _input = [np.max([np.min([_input[i], self.maxJ[i]]), self.minJ[i]]) for i in range(3)]
        elif self.flag == 2:
            _input = [np.max([np.min([_input[i], self.maxS[i]]), self.minS[i]]) for i in range(3)]
        else:
            pass
        return _input

    def state_update(self, new_input: list):
        new_input = self.input_saturation_check(new_input)
        if self.flag == 0:
            self.A = new_input  # new a
            self.P = [self.P[i] + self.V[i] * self.dt + 1 / 2 * self.A[i] * self.dt ** 2 for i in range(3)]
            self.V = [self.V[i] + self.dt * self.A[i] for i in range(3)]
        elif self.flag == 1:
            self.J = new_input  # new j
            self.P = [self.P[i] + self.V[i] * self.dt + 1 / 2 * self.A[i] * self.dt ** 2 + 1 / 6 * self.J[i] * self.dt ** 3 for i in range(3)]
            self.V = [self.V[i] + self.A[i] * self.dt + 1 / 2 * self.J[i] * self.dt ** 2 for i in range(3)]
            self.A = [self.A[i] + self.dt * self.J[i] for i in range(3)]
        elif self.flag == 2:
            self.S = new_input  # new s
            self.P = [self.P[i] + self.V[i] * self.dt + 1 / 2 * self.A[i] * self.dt ** 2 + 1 / 6 * self.J[i] * self.dt ** 3 + 1 / 24 * self.S[i] * self.dt ** 4 for i in range(3)]
            self.V = [self.V[i] + self.A[i] * self.dt + 1 / 2 * self.J[i] * self.dt ** 2 + 1 / 6 * self.S[i] * self.dt ** 3 for i in range(3)]
            self.A = [self.A[i] + self.dt * self.J[i] + 1 / 2 * self.S[i] * self.dt ** 2 for i in range(3)]
            self.J = [self.J[i] + self.dt * self.S[i] for i in range(3)]
        else:
            pass

        self.state_saturation_check()
        self.time += self.dt

    def random_input(self, flag, only_XY) -> list:
        num = 2 if only_XY else 3
        if flag == 'acc':
            res = []
            for i in range(num):
                res.append(np.random.uniform(self.minA[i], self.maxA[i], 1)[0])
            if num == 2:
                res.append(0.)
        elif flag == 'jerk':
            res = []
            for i in range(num):
                res.append(np.random.uniform(self.minJ[i], self.maxA[i], 1)[0])
            if num == 2:
                res.append(0.)
        elif flag == 'snap':
            res = []
            for i in range(num):
                res.append(np.random.uniform(self.minS[i], self.maxA[i], 1)[0])
            if num == 2:
                res.append(0.)
        else:
            res = [np.inf, np.inf, np.inf]

        return res

    def reset(self):
        self.P = self.initP
        self.V = self.initV
        self.A = self.initA
        self.J = self.initJ
        self.S = self.initS

        self.time = 0.0

        self.Px = [self.initP[0]]
        self.Py = [self.initP[1]]
        self.Pz = [self.initP[2]]

        self.Vx = [self.initV[0]]
        self.Vy = [self.initV[1]]
        self.Vz = [self.initV[2]]

        self.Ax = [self.initA[0]]
        self.Ay = [self.initA[1]]
        self.Az = [self.initA[2]]
        self.T = [self.time]

        if self.initJ is not None:
            self.Jx = [self.initJ[0]]
            self.Jy = [self.initJ[1]]
            self.Jz = [self.initJ[2]]
        else:
            self.Jx = None
            self.Jy = None
            self.Jz = None

        if self.initS is not None:
            self.Sx = [self.initS[0]]
            self.Sy = [self.initS[1]]
            self.Sz = [self.initS[2]]
        else:
            self.Sx = None
            self.Sy = None
            self.Sz = None

    def data_save(self, filename, filepath, is_save_to_file):
        if is_save_to_file:
            data = pd.DataFrame({
                'time:': self.T,
                'Px': self.Px,
                'Py': self.Py,
                'Pz': self.Pz,
                'Vx': self.Vx,
                'Vy': self.Vy,
                'Vz': self.Vz,
                'Ax': self.Ax,
                'Ay': self.Ay,
                'Az': self.Az,
                'Jx': self.Jx,
                'Jy': self.Jy,
                'Jz': self.Jz,
                'Sx': self.Sx,
                'Sy': self.Sy,
                'Sz': self.Sz,
            })
            data.to_csv(filepath + filename, index=False, sep=',')
        else:
            self.Px.append(self.P[0])
            self.Py.append(self.P[1])
            self.Pz.append(self.P[2])

            self.Vx.append(self.V[0])
            self.Vy.append(self.V[1])
            self.Vz.append(self.V[2])

            self.Ax.append(self.A[0])
            self.Ay.append(self.A[1])
            self.Az.append(self.A[2])

            self.T.append(self.time)

            if self.initJ is not None:
                self.Jx.append(self.J[0])
                self.Jy.append(self.J[1])
                self.Jz.append(self.J[2])

            if self.initS is not None:
                self.Sx.append(self.S[0])
                self.Sy.append(self.S[1])
                self.Sz.append(self.S[2])
