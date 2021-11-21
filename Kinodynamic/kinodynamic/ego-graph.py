import math

import numpy as np
import scipy as sp
from Map.Continuous.samplingmap import samplingmap
import cv2 as cv
import copy


def single_trajectory_pf(start, target, t_opt):
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    alpha1 = -3 * dx / t_opt ** 3
    beta1 = -alpha1 * t_opt
    alpha2 = -3 * dy / t_opt ** 3
    beta2 = -alpha2 * t_opt
    t_step = t_opt / 100
    t = 0
    [x, y] = map.start
    while t <= t_opt:
        x_new = alpha1 * t ** 3 / 6 + beta1 * t ** 2 / 2 + map.start[0]
        y_new = alpha2 * t ** 3 / 6 + beta2 * t ** 2 / 2 + map.start[1]
        cv.line(map.image, map.dis2pixel([x_new, y_new]), map.dis2pixel([x, y]), (0, 0, 255), 2)
        x = copy.deepcopy(x_new)
        y = copy.deepcopy(y_new)
        t += t_step
    cv.imshow(map.name4image, map.image)
    # cv.waitKey(0)

def single_trajectory_pfvf(start, target, vf, t_opt):
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    vx = vf[0]
    vy = vf[1]
    alpha1 = (6 * vx * t_opt - 12 * dx) / t_opt ** 3
    beta1 = (6 * dx - 2 * vx * t_opt) / t_opt ** 2
    alpha2 = (6 * vy * t_opt - 12 * dy) / t_opt ** 3
    beta2 = (6 * dy - 2 * vy * t_opt) / t_opt ** 2
    t_step = t_opt / 100
    t = 0
    [x, y] = map.start
    while t <= t_opt:
        x_new = alpha1 * t ** 3 / 6 + beta1 * t ** 2 / 2 + map.start[0]
        y_new = alpha2 * t ** 3 / 6 + beta2 * t ** 2 / 2 + map.start[1]
        cv.line(map.image, map.dis2pixel([x_new, y_new]), map.dis2pixel([x, y]), (0, 0, 255), 2)
        x = copy.deepcopy(x_new)
        y = copy.deepcopy(y_new)
        t += t_step
    cv.imshow(map.name4image, map.image)
    # cv.waitKey(0)


if __name__ == '__main__':
    map = samplingmap(width=400,
              height=400,
              x_size=10,
              y_size=10,
              image_name='samplingmap',
              start=[5, 5],     # 4.5, 8.5
              terminal=[10, 10],
              obs=None,
              map_file=None)
    '''ego-graph-pf'''
    # x_step = 1
    # y_step = 1
    # x = np.arange(0, map.x_size + x_step, x_step)
    # y = np.arange(0, map.y_size + y_step, y_step)
    # for xf in x:
    #     for yf in y:
    #         if (xf == map.start[0]) and (yf == map.start[1]):
    #             continue
    #         dx = xf - map.start[0]
    #         dy = yf - map.start[1]
    #         t_optimal = math.sqrt(3 * math.sqrt(dx ** 2 + dy ** 2))
    #         single_trajectory_pf(map.start, [xf, yf], t_optimal)
    # cv.imwrite('ego_graph_pf.png', map.image)

    '''ego-graph-pfvf'''
    x_step = 1
    y_step = 1
    x = np.arange(0, map.x_size + x_step, x_step)
    y = np.arange(0, map.y_size + y_step, y_step)
    vf = [-1, -1]
    for xf in x:
        for yf in y:
            if (xf == map.start[0]) and (yf == map.start[1]):
                continue
            dx = xf - map.start[0]
            dy = yf - map.start[1]
            vx = vf[0]
            vy = vf[1]
            t4 = 1
            t3 = 0
            t2 = 4 * (vx ** 2 + vy ** 2)
            t1 = 12 * (vx * dx + vy * dy)
            t0 = 12 * (dx ** 2 + dy ** 2)
            J = np.inf
            t_opt = 0
            roots = np.roots([t4, t3, -t2, -2 * t1, -3 * t0])
            real = roots.real
            imag = roots.imag
            for i in range(4):
                if (imag[i] == 0) and (real[i] >= 0):
                    t = real[i]
                    J_temp = t + t2 / t + t1 / t ** 2 + t0 / t ** 3
                    if J_temp < J:
                        J = J_temp
                        t_opt = t
            single_trajectory_pfvf(map.start, [xf, yf], vf, t_opt)
    cv.imwrite('ego_graph_pfvf.png', map.image)

