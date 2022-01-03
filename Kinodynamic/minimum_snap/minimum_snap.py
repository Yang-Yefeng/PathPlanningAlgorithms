# import numpy as np
import sympy as sym
import cvxopt as cp
from Map import *
import pandas as pd
import matplotlib.pyplot as plt


def get_nodes_4_mini_jerk_using_opencv_callback(_map: samplingmap) -> list:
    nodes = []
    node_max = 10
    enough = False

    def callback(event, x, y, flags, param):
        if len(nodes) >= node_max:
            print('Enough...')
        if event == cv.EVENT_LBUTTONUP:  # 鼠标左键抬起
            point = _map.pixel2dis((x, y))
            cv.circle(_map.image, (x, y), 2, Color().DarkMagenta, -1)
            if min(point) <= 0. or point[0] > _map.x_size or point[1] > _map.y_size:  # out
                print('Out...')
            else:
                nodes.append(point)
                cv.putText(_map.image, str([round(point[0], 2), round(point[1], 2)]), (x + 5, y + 5), cv.FONT_HERSHEY_SIMPLEX,
                           0.4, Color().DarkMagenta, 1, cv.LINE_AA)
        elif event == cv.EVENT_RBUTTONUP:  # 鼠标右键抬起
            if len(nodes) > 0:
                drop = nodes.pop(-1)
                cv.circle(_map.image, _map.dis2pixel(drop), 3, Color().White, -1)
                drop_pixel = _map.dis2pixel(drop)
                cv.putText(_map.image, str([round(drop[0], 2), round(drop[1], 2)]), (drop_pixel[0] + 5, drop_pixel[1] + 5), cv.FONT_HERSHEY_SIMPLEX,
                           0.4, Color().White, 3, cv.LINE_AA)
            else:
                print('nodes in empty, nothing to delete...')

    cv.setMouseCallback(_map.name4image, callback)
    print('Please set nodes...')
    while True:
        cv.imshow(_map.name4image, _map.image)
        if len(nodes) >= node_max:
            enough = True
        if (cv.waitKey(1) == ord('q')) or enough:
            if len(nodes) >= 2:
                break
    cv.destroyAllWindows()
    return nodes


def draw_trajectory_cv(_map: samplingmap, x: list, y: list, draw: bool):
    assert len(x) == len(y)
    assert len(x) > 0
    assert len(x[0]) == len(y[0])
    for i in range(len(x)):
        color = Color().random_color_by_BGR()
        for j in range(len(x[i]) - 1):
            pt1_int = _map.dis2pixel([x[i][j], y[i][j]])
            pt2_int = _map.dis2pixel([x[i][j + 1], y[i][j + 1]])
            cv.line(_map.image, pt1_int, pt2_int, color, 2)
    cv.imwrite('minimum_snap.png', _map.image)
    if draw:
        cv.imshow(_map.name4image, _map.image)
        cv.waitKey(0)


def draw_trajectory_plt(_x: list, _y: list, _pts_x: list, _pts_y: list):
    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.title('minimum snap')
    plt.grid(True)
    for i in range(len(_x)):
        plt.plot(_x[i], _y[i])
    plt.plot(_pts_x, _pts_y, 'ro')
    plt.show()


class minimum_snap:
    """all inputs including position velocity acceleration should be one-dimensional"""
    def __init__(self,
                 poly_order=5,
                 points=None,
                 init_v=0.0,
                 init_a=0.0,
                 ter_v=0.0,
                 ter_a=0.0):
        assert poly_order >= 5 and points
        self.order = poly_order
        self.points = points
        self.init_v, self.init_a, self.ter_v, self.ter_a = init_v, init_a, ter_v, ter_a
        '''symbol coefficient vector of p, v, a, j, and s'''
        self.t = sym.symbols('t')
        self.t0 = sym.symbols('t0')
        self.tf = sym.symbols('tf')
        self.p_sym = sym.Matrix(1, self.order + 1, [self.t**i for i in range(self.order + 1)])      # symbol for position
        self.v_sym = self.p_sym.diff(self.t, 1)                                  # symbol for velocity
        self.a_sym = self.p_sym.diff(self.t, 2)                                  # symbol for acceleration
        self.j_sym = self.p_sym.diff(self.t, 3)                                  # symbol for jerk
        self.s_sym = self.p_sym.diff(self.t, 4)                                  # symbol for snap
        '''symbol coefficient vector of p, v, a, j, and s'''

        self.n_points = len(self.points)
        self.n_trajectory = self.n_points - 1  # number of the trajectory
        self.time_per_trajectory = self.__time_allocation()
        self.time_per_node = self.__time_traj2node()

        self.Qn = sym.integrate(self.s_sym.T.multiply(self.s_sym), (self.t, self.t0, self.tf))       # cost matrix of one trajectory
        self.Q = cp.matrix()
        self.P = cp.matrix()
        self.A = cp.matrix()
        self.B = cp.matrix()
        self.G = cp.matrix()
        self.H = cp.matrix()

        self.coefficient = [[0.0 for _ in range(self.order + 1)] for _ in range(self.n_trajectory)]

    def __time_allocation(self):
        time = []
        for i in range(self.n_trajectory):
            time.append(2.000)
        return time

    def __time_traj2node(self):
        time = [0]
        for i in range(self.n_trajectory):
            time.append(time[i] + self.time_per_trajectory[i])
        return time

    def node_time_info(self):
        print('nodes:', np.round(self.points, 4))
        print('time node:', self.time_per_node)
        print('time traj:', self.time_per_trajectory)

    @staticmethod
    def __sympy_2_float_list(sympy_matrix: sym.Matrix):
        value = sympy_matrix.tolist()
        if len(value) == 1:
            return [float(_i) for _i in value[0]]
        else:
            return [[float(_i) for _i in _j] for _j in value]

    def get_Q(self):
        Q_cell = []
        for i in range(self.n_trajectory):      # use relative time
            value = self.Qn.subs([(self.t0, 0.0), (self.tf, self.time_per_trajectory[i])])
            list_value = self.__sympy_2_float_list(value)
            Q_cell.append(cp.matrix(list_value))
        self.Q = 2.00 * cp.spdiag(Q_cell)   # 乘以 2 是为了与 cvxopt 中的 minimum cost 的代价函数格式相统一
        # print(self.Q)
        print('Q size:', self.Q.size)

    def get_P(self):
        self.P = cp.matrix(np.array(np.zeros(((self.order + 1) * self.n_trajectory, 1)), dtype=float), ((self.order + 1) * self.n_trajectory, 1))
        print('P size:', self.P.size)

    def get_coefficient_cell2(self, time: float, flag: str) -> list:
        """
        :param time:        time of the cell
        :param flag:        'position' for Cp, 'velocity' for Cv, 'acceleration for Ca', 'jerk' for Cj, 'snap' for Cs
        :return:            block cell
        :tips:              time must be float
        """
        if flag == 'position':
            value = self.p_sym.subs([(self.t, time)])
        elif flag == 'velocity':
            value = self.v_sym.subs([(self.t, time)])
        elif flag == 'acceleration':
            value = self.a_sym.subs([(self.t, time)])
        elif flag == 'jerk':
            value = self.j_sym.subs([(self.t, time)])
        elif flag == 'snap':
            value = self.s_sym.subs([(self.t, time)])
        else:
            value = []
        list_value = self.__sympy_2_float_list(value)
        return list_value

    def copy_coefficient_cell2A(self, C: list, cell_row: int, cell_col: int):
        """
        :param C:               the cell get from 'get_coefficient_cell2'
        :param cell_row:        the row of this cell in A
        :param cell_col:        the col of this cell in A
        :return:                None
        :tips:                  cell_col 是分块矩阵的列数，0 就是 0:6，1 就是 6: 12，2 就是 12:18，以此类推！！！！！！
        """
        self.A[cell_row, (self.order + 1) * cell_col: (self.order + 1) * (cell_col + 1)] = copy.deepcopy(C)

    def get_A(self):
        zeros = [[0.00 for _ in range(4 * self.n_points - 2)] for _ in range((self.order + 1) * self.n_trajectory)]
        self.A = cp.matrix(zeros, (4 * self.n_points - 2, (self.order + 1) * self.n_trajectory))
        row_index = 0
        """将多项式系数矩阵复制进A中"""
        '''三个初始点位置，速度，加速度系数矩阵加入A中'''
        cellp = self.get_coefficient_cell2(time=self.time_per_node[0], flag='position')
        cellv = self.get_coefficient_cell2(time=self.time_per_node[0], flag='velocity')
        cella = self.get_coefficient_cell2(time=self.time_per_node[0], flag='acceleration')
        self.copy_coefficient_cell2A(C=cellp, cell_row=row_index, cell_col=0)
        row_index += 1
        self.copy_coefficient_cell2A(C=cellv, cell_row=row_index, cell_col=0)
        row_index += 1
        self.copy_coefficient_cell2A(C=cella, cell_row=row_index, cell_col=0)
        row_index += 1
        '''三个初始点位置，速度，加速度系数矩阵加入A中'''

        '''N-2个中间点的位置系数矩阵加入A中'''
        for i in range(self.n_points):
            if i != 0 and i != self.n_points - 1:
                cellp = self.get_coefficient_cell2(time=self.time_per_node[i], flag='position')
                self.copy_coefficient_cell2A(C=cellp, cell_row=row_index, cell_col=i - 1)
                row_index += 1
                self.copy_coefficient_cell2A(C=cellp, cell_row=row_index, cell_col=i)
                row_index += 1
        '''N-2个中间点的位置系数矩阵加入A中'''

        '''将末位点位置，速度，加速度系数矩阵加入A中'''
        cellp = self.get_coefficient_cell2(time=self.time_per_node[-1], flag='position')
        cellv = self.get_coefficient_cell2(time=self.time_per_node[-1], flag='velocity')
        cella = self.get_coefficient_cell2(time=self.time_per_node[-1], flag='acceleration')
        self.copy_coefficient_cell2A(C=cellp, cell_row=row_index, cell_col=self.n_trajectory - 1)
        row_index += 1
        self.copy_coefficient_cell2A(C=cellv, cell_row=row_index, cell_col=self.n_trajectory - 1)
        row_index += 1
        self.copy_coefficient_cell2A(C=cella, cell_row=row_index, cell_col=self.n_trajectory - 1)
        row_index += 1
        '''将末位点位置，速度，加速度系数矩阵加入A中'''

        '''其余行依次加入N-2个中间点在前后两条线段的速度，加速度相等'''
        for i in range(self.n_points):
            if i != 0 and i != self.n_points - 1:
                cellv = self.get_coefficient_cell2(time=self.time_per_node[i], flag='velocity')
                cella = self.get_coefficient_cell2(time=self.time_per_node[i], flag='acceleration')
                self.copy_coefficient_cell2A(C=cellv, cell_row=row_index, cell_col=i - 1)
                self.copy_coefficient_cell2A(C=list(-np.array(cellv)), cell_row=row_index, cell_col=i)
                row_index += 1
                self.copy_coefficient_cell2A(C=cella, cell_row=row_index, cell_col=i - 1)
                self.copy_coefficient_cell2A(C=list(-np.array(cella)), cell_row=row_index, cell_col=i)
                row_index += 1
        '''其余行依次加入N-2个中间点在前后两条线段的速度，加速度相等'''
        """将多项式系数矩阵复制进A中"""
        assert row_index == self.A.size[0]
        print('A Size:', self.A.size)

    def get_B(self):
        b = []
        for i in range(self.n_points):
            if i == 0:
                b.append(float(self.points[0]))
                b.append(float(self.init_v))
                b.append(float(self.init_a))
            elif i == self.n_points - 1:
                b.append(float(self.points[-1]))
                b.append(float(self.ter_v))
                b.append(float(self.ter_a))
            else:
                b.append(float(self.points[i]))
                b.append(float(self.points[i]))
        for i in range(self.n_points - 2):
            b.append(0.00)
            b.append(0.00)
        # print(b)
        self.B = cp.matrix(np.array(b, dtype=float), (4 * self.n_points - 2, 1))
        print('B size:', self.B.size)

    def save_matrices2csv(self, path):
        pd.DataFrame([list(self.Q[i, :]) for i in range(self.Q.size[0])]).to_csv(path + 'Q.csv', encoding='gbk', header=False, index=False)
        pd.DataFrame([list(self.P[i, :]) for i in range(self.P.size[0])]).to_csv(path + 'P.csv', encoding='gbk', header=False, index=False)
        pd.DataFrame([list(self.A[i, :]) for i in range(self.A.size[0])]).to_csv(path + 'A.csv', encoding='gbk', header=False, index=False)
        pd.DataFrame([list(self.B[i, :]) for i in range(self.B.size[0])]).to_csv(path + 'B.csv', encoding='gbk', header=False, index=False)
        pd.DataFrame(self.coefficient).to_csv(path + 'coefficient.csv', encoding='gbk', header=False, index=False)

    def minimum_snap(self):
        self.node_time_info()
        self.get_Q()
        self.get_P()
        self.get_A()
        self.get_B()
        sol = cp.solvers.qp(self.Q, self.P, None, None, self.A, self.B)
        for i in range(self.n_trajectory):
            self.coefficient[i] = sol['x'][i * (self.order + 1): (i + 1) * (self.order + 1)]

    def get_trajectory_points(self, points_per_trajectory: int = 100) -> list:
        trajectory = []
        for i in range(self.n_trajectory):      # i 是轨迹编号
            t = np.linspace(self.time_per_node[i], self.time_per_node[i + 1], points_per_trajectory)
            sub_trajectory = np.zeros((1, points_per_trajectory)).squeeze()
            for j in range(self.order + 1):     # j 是 time 的次数
                sub_trajectory += self.coefficient[i][j] * t ** j
            trajectory.append(list(sub_trajectory))
        return trajectory


if __name__ == '__main__':
    snap_map = samplingmap(width=400, height=400, x_size=10, y_size=10, image_name='minimum_snap', start=None, terminal=None, obs=None, map_file=None)
    pts = get_nodes_4_mini_jerk_using_opencv_callback(snap_map)     # get nodes
    # pts = [[3, 3], [3, 5], [5, 5], [5, 3]]
    v0, a0, vf, af = [0, 0], [0, 0], [0, 0], [0, 0]                 # initial velocity and acceleration

    '''separate x and y'''
    pts_x = [i[0] for i in pts]
    pts_y = [i[1] for i in pts]
    '''separate x and y'''

    '''minimum snap in x'''
    snap_x = minimum_snap(poly_order=5, points=pts_x, init_v=v0[0], init_a=a0[0], ter_v=vf[0], ter_a=af[0])
    snap_x.minimum_snap()
    snap_x.save_matrices2csv(path='snap_x/')
    trajectory_x = snap_x.get_trajectory_points()
    '''minimum snap in x'''

    '''minimum snap in y'''
    snap_y = minimum_snap(poly_order=5, points=pts_y, init_v=v0[1], init_a=a0[1], ter_v=vf[1], ter_a=af[1])
    snap_y.minimum_snap()
    snap_x.save_matrices2csv(path='snap_y/')
    trajectory_y = snap_y.get_trajectory_points()
    '''minimum snap in y'''

    '''draw in opencv'''
    draw_trajectory_cv(snap_map, trajectory_x, trajectory_y, True)
    '''draw in opencv'''

    # '''draw in plt'''
    # draw_trajectory_plt(trajectory_x, trajectory_y, pts_x, pts_y)
    # '''draw in plt'''
