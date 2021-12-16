import math
import numpy as np
import cvxopt as cp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd


omega = 0.08


def Conic_spiral_equation(_omega: float, t: float, v: float, h: float) -> tuple:
    """
    :param _omega:       angular velocity
    :param t:           time
    :param v:           velocity
    :param h:           initial height
    :return:
    """
    _px = v * t * math.cos(_omega * t)
    _py = v * t * math.sin(_omega * t)
    _pz = v * t + h
    _vx = -_omega * v * t * math.sin(_omega * t)
    _vy = _omega * v * t * math.cos(_omega * t)
    _vz = v
    _ax = -_omega ** 2 * v * t * math.cos(_omega * t)
    _ay = -_omega ** 2 * v * t * math.sin(_omega * t)
    _az = 0.0
    return _px, _py, _pz, _vx, _vy, _vz, _ax, _ay, _az


class Linear_MPC:
    def __init__(self):
        self.v_max = [6, 6, 6]
        self.v_min = [-6, -6, -1]
        self.a_max = [3, 3, 3]
        self.a_min = [-3, -3, -1]
        self.j_max = [3, 3, 2]
        self.j_min = [-3, -3, -2]
        self.t_predict = 4.0
        self.dt = 0.2
        self.K = int(self.t_predict / self.dt)

        self.p0 = [0.0, 8.0, 20.0]
        self.v0 = [0.0, 0.0, 0.0]
        self.a0 = [0.0, 0.0, 0.0]

    def getPredictionMatrix(self, p0, v0, a0):  # input data should be One-dimensional
        K = int(self.t_predict / self.dt)
        Tp = cp.matrix(np.array(np.zeros((K, K)), dtype=float), (K, K))
        Tv = cp.matrix(np.array(np.zeros((K, K)), dtype=float), (K, K))
        Ta = cp.matrix(np.array(np.zeros((K, K)), dtype=float), (K, K))
        Bp = cp.matrix(np.array(np.ones((K, 1)) * p0, dtype=float), (K, 1))
        Bv = cp.matrix(np.array(np.ones((K, 1)) * v0, dtype=float), (K, 1))
        Ba = cp.matrix(np.array(np.ones((K, 1)) * a0, dtype=float), (K, 1))
        for i in range(K):
            Ta[i, 0: i + 1] = np.ones((1, i + 1), dtype=float) * self.dt
        for i in range(K):
            for j in range(i):
                Tv[i, j] = (i - j + 0.5) * self.dt ** 2
        for i in range(K):
            for j in range(i):
                Tp[i, j] = ((i - j + 1) * (i - j) / 2 + 1 / 6) * self.dt ** 3
        for i in range(K):
            Bv[i] = Bv[i] + (i + 1) * self.dt * a0
            Bp[i] = Bp[i] + (i + 1) * self.dt * v0 + (i + 1) ** 2 / 2 * a0 * self.dt ** 2
        return Tp, Tv, Ta, Bp, Bv, Ba

    def get_Q_P_inCostFunction_with_ref(self, w1, w2, w3, w4, tstart, index: int):
        Tp, Tv, Ta, Bp, Bv, Ba = self.getPredictionMatrix(self.p0[index], self.v0[index], self.a0[index])
        p_ref = cp.matrix(np.array(np.zeros((self.K, 1)), dtype=float), (self.K, 1))
        v_ref = cp.matrix(np.array(np.zeros((self.K, 1)), dtype=float), (self.K, 1))
        a_ref = cp.matrix(np.array(np.zeros((self.K, 1)), dtype=float), (self.K, 1))
        for i in range(self.K):
            res = Conic_spiral_equation(_omega=omega, t=tstart + i * self.dt, v=-0.5, h=20)
            p_ref = res[index]
            v_ref = res[index + 3]
            a_ref = res[index + 6]
        Q = 2.0 * (w1 * Tp.T * Tp + w2 * Tv.T * Tv + w3 * Ta.T * Ta + w4 * cp.matrix(np.ones((self.K, self.K), dtype=float), (self.K, self.K)))
        P = cp.matrix(2.0 * (w1 * (Bp - p_ref).T * Tp + w2 * (Bv - v_ref).T * Tv + w3 * (Ba - a_ref).T * Ta)).T
        return Q, P

    def get_G_H_inequalityConstraints(self, index: int):
        Tp, Tv, Ta, Bp, Bv, Ba = self.getPredictionMatrix(self.p0[index], self.v0[index], self.a0[index])
        G_v_max = Tv
        H_v_max = cp.matrix(np.array(np.ones((self.K, 1)) * self.v_max[index], dtype=float), (self.K, 1)) - Bv
        G_v_min = -Tv
        H_v_min = Bv - cp.matrix(np.array(np.ones((self.K, 1)) * self.v_min[index], dtype=float), (self.K, 1))
        G_a_max = Ta
        H_a_max = cp.matrix(np.array(np.ones((self.K, 1)) * self.a_max[index], dtype=float), (self.K, 1)) - Ba
        G_a_min = -Ta
        H_a_min = Ba - cp.matrix(np.array(np.ones((self.K, 1)) * self.a_min[index], dtype=float), (self.K, 1))
        G_j_max = cp.matrix(np.array(np.eye(self.K), dtype=float), (self.K, self.K))
        H_j_max = cp.matrix(np.array(np.ones((self.K, 1)) * self.j_max[index], dtype=float), (self.K, 1))
        G_j_min = -cp.matrix(np.array(np.eye(self.K), dtype=float), (self.K, self.K))
        H_j_min = -cp.matrix(np.array(np.ones((self.K, 1)) * self.j_min[index], dtype=float), (self.K, 1))
        G = cp.matrix([G_v_max, G_v_min, G_a_max, G_a_min, G_j_max, G_j_min])
        H = cp.matrix([H_v_max, H_v_min, H_a_max, H_a_min, H_j_max, H_j_min])
        return G, H


if __name__ == '__main__':
    mpc = Linear_MPC()
    t_teminal = 400
    time = np.arange(0, t_teminal + mpc.dt, mpc.dt)
    px, py, pz = [], [], []
    vx, vy, vz = [], [], []
    ax, ay, az = [], [], []
    jx, jy, jz = [], [], []
    prefX, prefY, prefZ = [], [], []
    for _t in time:
        xx, yy, zz, _, _, _, _, _, _ = Conic_spiral_equation(omega, _t, -0.5, 20)
        prefX.append(xx)
        prefY.append(yy)
        prefZ.append(zz)
        Qx, Px = mpc.get_Q_P_inCostFunction_with_ref(1.0, 1.0, 1.0, 1.0, _t, 0)
        Qy, Py = mpc.get_Q_P_inCostFunction_with_ref(1.0, 1.0, 1.0, 1.0, _t, 1)
        Qz, Pz = mpc.get_Q_P_inCostFunction_with_ref(1.0, 1.0, 1.0, 1.0, _t, 2)
        inequality = False
        if inequality:
            Gx, Hx = mpc.get_G_H_inequalityConstraints(0)
            Jx = cp.solvers.qp(Qx, Px, Gx, Hx, None, None)
            jerkX = Jx['x'][0]

            Gy, Hy = mpc.get_G_H_inequalityConstraints(1)
            Jy = cp.solvers.qp(Qy, Py, Gy, Hy, None, None)
            jerkY = Jy['x'][0]

            Gz, Hz = mpc.get_G_H_inequalityConstraints(2)
            Jz = cp.solvers.qp(Qz, Pz, Gz, Hz, None, None)
            jerkZ = Jz['x'][0]
        else:
            Jx = cp.solvers.qp(Qx, Px, None, None, None, None)
            jerkX = Jx['x'][0]

            Jy = cp.solvers.qp(Qy, Py, None, None, None, None)
            jerkY = Jy['x'][0]

            Jz = cp.solvers.qp(Qz, Pz, None, None, None, None)
            jerkZ = Jz['x'][0]

        mpc.p0[0] = mpc.p0[0] + mpc.v0[0] * mpc.dt + 0.5 * mpc.a0[0] * mpc.dt ** 2 + jerkX * mpc.dt ** 3 / 6
        mpc.p0[1] = mpc.p0[1] + mpc.v0[1] * mpc.dt + 0.5 * mpc.a0[1] * mpc.dt ** 2 + jerkY * mpc.dt ** 3 / 6
        mpc.p0[2] = mpc.p0[2] + mpc.v0[2] * mpc.dt + 0.5 * mpc.a0[2] * mpc.dt ** 2 + jerkZ * mpc.dt ** 3 / 6

        mpc.v0[0] = mpc.v0[0] + mpc.a0[0] * mpc.dt + 0.5 * jerkX * mpc.dt ** 2
        mpc.v0[1] = mpc.v0[1] + mpc.a0[1] * mpc.dt + 0.5 * jerkY * mpc.dt ** 2
        mpc.v0[2] = mpc.v0[2] + mpc.a0[2] * mpc.dt + 0.5 * jerkZ * mpc.dt ** 2

        mpc.a0[0] = mpc.a0[0] + jerkX * mpc.dt
        mpc.a0[1] = mpc.a0[1] + jerkY * mpc.dt
        mpc.a0[2] = mpc.a0[2] + jerkZ * mpc.dt

        px.append(mpc.p0[0])
        py.append(mpc.p0[1])
        pz.append(mpc.p0[2])

        vx.append(mpc.v0[0])
        vy.append(mpc.v0[1])
        vz.append(mpc.v0[2])

        ax.append(mpc.a0[0])
        ay.append(mpc.a0[1])
        az.append(mpc.a0[2])

        jx.append(jerkX)
        jy.append(jerkY)
        jz.append(jerkZ)

    dataframe = pd.DataFrame({'time': time,
                              'px': px,
                              'py': py,
                              'pz': pz,
                              'vx': vx,
                              'vy': vy,
                              'vz': vz,
                              'ax': ax,
                              'ay': ay,
                              'az': az,
                              'jx': jx,
                              'jy': jy,
                              'jz': jz})
    dataframe.to_csv("MPC_linear_track.csv", index=False, sep=',')
    # plot
    fig1 = plt.figure()
    ax1 = plt.axes(projection='3d')
    ax1.plot(px, py, pz, 'r')
    ax1.plot(prefX, prefY, prefZ, 'b')
    plt.show()

    # fig2 = plt.figure()
    # plt.plot(t, vx, 'r')
    # plt.plot(t, vy, 'g')
    # plt.plot(t, vz, 'b')
    # plt.show()
