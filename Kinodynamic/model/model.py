import numpy as np


class Model:
    def __init__(self, model_l, model_w, radius, A, B, s_dimension, p_dimension, u_dimension, s_init, time_step):
        self.model_l = model_l
        self.model_w = model_w
        self.radius = radius
        self.s_dimension = s_dimension
        self.u_dimension = u_dimension
        self.p_dimension = p_dimension
        # self.s = np.array(s_init).T
        self.s = s_init
        self.A = A      # np.array(A)
        self.B = B      # np.array(B)
        self.time = 0.
        self.time_step = time_step
        self.p = self.s[0: p_dimension]
        self.v = self.s[p_dimension: p_dimension + 2]
        # self.initial_para = [self.s, self.A, self.B]
        if s_dimension == 2 * p_dimension:
            self.a = [0 for _ in range(p_dimension)]
            self.jerk = None
        else:
            self.a = self.s[p_dimension + 2: p_dimension + 4]
            self.jerk = [0 for _ in range(p_dimension)]

    def differential_equation(self, x, u):
        return np.dot(self.A, x) + np.dot(self.B, u)

    def state_update_single(self, u):
        ministep = 10
        h = self.time_step / ministep
        for i in range(ministep):
            K1 = self.differential_equation(self.s, u)
            K2 = self.differential_equation(self.s + h * K1 / 2, u)
            K3 = self.differential_equation(self.s + h * K2 / 2, u)
            K4 = self.differential_equation(self.s + h * K3, u)
            self.s = self.s + h * (K1 + 2 * K2 + 2 * K3 + K4) / 6
            self.time += h
        self.p = self.s[0: self.p_dimension]
        self.v = self.s[self.p_dimension: self.p_dimension + 2]
        if self.s_dimension == 2 * self.p_dimension:
            self.a = u
            self.jerk = None
        else:
            self.a = self.s[self.p_dimension + 2: self.p_dimension + 4]
            self.jerk = u


    def state_update(self, u, time_interval):
        for i in range(int(time_interval / self.time_step)):
            self.state_update_single(u[i])

    def reset(self, s_dimension, p_dimension, s_init):
        self.s = s_init
        self.time = 0.
        self.p = self.s[0: p_dimension]
        self.v = self.s[p_dimension: p_dimension + 2]
        # self.initial_para = [self.s, self.A, self.B]
        if s_dimension == 2 * p_dimension:
            self.a = [0 for _ in range(p_dimension)]
            self.jerk = None
        else:
            self.a = self.s[p_dimension + 2: p_dimension + 4]
            self.jerk = [0 for _ in range(p_dimension)]
