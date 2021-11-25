from Map import *


def sind(theta):
    return math.sin(theta / 180.0 * math.pi)

def cosd(theta):
    return math.cos(theta / 180.0 * math.pi)


class KinoDynamic(samplingmap):
    def __init__(self, _map_parameters):
        samplingmap.__init__(self,
                             _map_parameters['width'],
                             _map_parameters['height'],
                             _map_parameters['x_size'],
                             _map_parameters['y_size'],
                             _map_parameters['image_name'],
                             _map_parameters['start'],
                             _map_parameters['terminal'],
                             _map_parameters['obs'],
                             _map_parameters['map_file'])

    @staticmethod
    def root4cost_function(theta: list) -> tuple:
        roots = np.roots([theta[4], theta[3], -theta[2], -2 * theta[1], -3 * theta[0]])
        real = roots.real
        imag = roots.imag
        J = np.inf
        t_opt = 0
        for i in range(4):
            if (imag[i] == 0) and (real[i] >= 0):
                t = real[i]
                J_temp = theta[4] * t + theta[3] + theta[2] / t + theta[1] / t ** 2 + theta[0] / t ** 3
                if J_temp < J:
                    J = J_temp
                    t_opt = t
        return t_opt, J

    def cal_final_state_pf_fixed(self, p0, dp, v0, T, is_dynamic_draw):
        alpha1 = 3 * (v0[0] * T - dp[0]) / T ** 3
        beta1 = -alpha1 * T
        alpha2 = 3 * (v0[1] * T - dp[1]) / T ** 3
        beta2 = -alpha2 * T
        axf = alpha1 * T + beta1
        ayf = alpha2 * T + beta2
        vxf = alpha1 * T ** 2 / 2 + beta1 * T + v0[0]
        vyf = alpha2 * T ** 2 / 2 + beta2 * T + v0[1]

        t_step = T / 100
        _t = 0
        [_x, _y] = p0
        color = Color().random_color_by_BGR()
        while _t <= T:
            x_new = alpha1 * _t ** 3 / 6 + beta1 * _t ** 2 / 2 + v0[0] * _t + p0[0]
            y_new = alpha2 * _t ** 3 / 6 + beta2 * _t ** 2 / 2 + v0[1] * _t + p0[1]
            cv.line(self.image, self.dis2pixel([x_new, y_new]), self.dis2pixel([_x, _y]), color, 1)
            _x = copy.deepcopy(x_new)
            _y = copy.deepcopy(y_new)
            _t += t_step
        if is_dynamic_draw:
            cv.imshow(self.name4image, self.image)
            cv.waitKey(0)
        return axf, ayf, vxf, vyf

    def cal_final_state_pf_vf_fixed(self, p0, dp, v0, dv, T, is_dynamic_draw):
        alpha1 = (12 * v0[0] * T + 6 * dv[0] * T - 12 * dp[0]) / T ** 3
        beta1 = (6 * dp[0] - 6 * v0[0] * T - 2 * dv[0] * T) / T ** 2
        alpha2 = (12 * v0[1] * T + 6 * dv[1] * T - 12 * dp[1]) / T ** 3
        beta2 = (6 * dp[1] - 6 * v0[1] * T - 2 * dv[1] * T) / T ** 2
        axf = alpha1 * T + beta1
        ayf = alpha2 * T + beta2

        t_step = T / 100
        _t = 0
        [_x, _y] = p0
        color = Color().random_color_by_BGR()
        while _t <= T:
            x_new = alpha1 * _t ** 3 / 6 + beta1 * _t ** 2 / 2 + v0[0] * _t + p0[0]
            y_new = alpha2 * _t ** 3 / 6 + beta2 * _t ** 2 / 2 + v0[1] * _t + p0[1]
            cv.line(self.image, self.dis2pixel([x_new, y_new]), self.dis2pixel([_x, _y]), color, 1)
            _x = copy.deepcopy(x_new)
            _y = copy.deepcopy(y_new)
            _t += t_step
        if is_dynamic_draw:
            cv.imshow(self.name4image, self.image)
            cv.waitKey(0)
        return axf, ayf

    def solve_OBVP_4_2D_simplify_UAV(self, p0, pf, v0, vf, tf, is_dynamic_draw):
        """
        :param is_dynamic_draw:
        :param p0:      initial position
        :param pf:      terminal position
        :param v0:      initial velocity
        :param vf:      terminal velocity
        :param v0:      initial time
        :param tf:      terminal time
        :return:        optimal items need to be solved, T is the relative, not absolute.
        """
        axf, ayf, vxf, vyf, pxf, pyf, T, J_min = np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf
        # pxf = pf[0]
        # pyf = pf[1]
        if tf is None:      # that means terminal time is free
            dx = pf[0] - p0[0]
            dy = pf[1] - p0[1]
            if vf is None:  # that means terminal velocity is free
                theta0 = 3 * (dx ** 2 + dy ** 2)
                theta1 = -6 * (v0[0] * dx + v0[1] * dy)
                theta2 = 3 * (v0[0] ** 2 + v0[1] ** 2)
                theta3 = 0
                theta4 = 1
                theta = [theta0, theta1, theta2, theta3, theta4]
                T, J_min = self.root4cost_function(theta)
                axf, ayf, vxf, vyf = self.cal_final_state_pf_fixed(p0, [dx, dy], v0, T, is_dynamic_draw)
                pxf = pf[0]
                pyf = pf[1]
            else:           # that means terminal velocity is fixed
                # TODO 昨晚的草稿纸我扔哪去了......T_T
                # TODO 2021.11.23 重新算，这次对了，一定要用Matlab，没有必要用手^_^
                dvx = vf[0] - v0[0]
                dvy = vf[1] - v0[1]
                theta0 = 12 * (dx ** 2 + dy ** 2)
                theta1 = -(12 * (dvx * dx + dvy * dy) + 24 * (dx * v0[0] + dy * v0[1]))
                theta2 = 4 * (dvx ** 2 + dvy ** 2) + 12 * (dvx * v0[0] + dvy * v0[1] + v0[0] ** 2 + v0[1] ** 2)
                theta3 = 0
                theta4 = 1
                theta = [theta0, theta1, theta2, theta3, theta4]
                T, J_min = self.root4cost_function(theta)
                axf, ayf = self.cal_final_state_pf_vf_fixed(p0, [dx, dy], v0, [dvx, dvy], T, is_dynamic_draw)
                vxf = vf[0]
                vyf = vf[1]
                pxf = pf[0]
                pyf = pf[1]
        else:               # that means terminal time is fixed
            pass

        return [axf, ayf, vxf, vyf, pxf, pyf, T, J_min]

    def draw_ego_graph(self, p0, v0, vf, is_dynamic_draw, x_step, y_step):
        """
        :param y_step:
        :param x_step:
        :param p0:                  initial position [x0, y0]
        :param v0:                  initial velocity [v0, y0]
        :param vf:                  terminal velocity [vxf, vyf]
        :param is_dynamic_draw:     draw dynamic process
        :return:                    None
        """
        _x = np.arange(0, kino.x_size + x_step, x_step)
        _y = np.arange(0, kino.y_size + y_step, y_step)
        for xf in _x:
            for yf in _y:
                if (xf == kino.start[0]) and (yf == kino.start[1]):
                    continue
                self.solve_OBVP_4_2D_simplify_UAV(p0=p0,
                                                  pf=[xf, yf],
                                                  v0=v0,
                                                  vf=vf,
                                                  tf=None,
                                                  is_dynamic_draw=is_dynamic_draw)
        name = './ego_graph/' + \
               'ego_graph' + \
               '-p0' + str(p0) + \
               '-v0' + str(v0) + \
               '-vf' + str(vf) + '.png'
        cv.imwrite(name, self.image)


if __name__ == '__main__':
    map_parameters = {
        'width': 500,
        'height': 500,
        'x_size': 10,
        'y_size': 10,
        'image_name': 'kinodynamic',
        'start': [5, 5],
        'terminal': [10, 10],
        'obs': None,
        'map_file': None
    }

    kino = KinoDynamic(map_parameters)
    kino.draw_ego_graph(p0=[5, 5], v0=[0, 0], vf=[-1, 1], is_dynamic_draw=False, x_step=1, y_step=1)
