import random

import cv2 as cv
import os
import sys
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../PathPlanningAlgorithms/")

from Map.Color.Color import Color
from Map.Continuous.samplingmap import samplingmap


def sind(theta):
    return math.sin(theta / 180.0 * math.pi)


def cosd(theta):
    return math.cos(theta / 180.0 * math.pi)


class rasterizedmap:
    def __init__(self, _samplingmap: samplingmap, x_grid: int, y_grid: int):
        self.sampling_map = _samplingmap
        self.x_grid = x_grid  # x栅格数
        self.y_grid = y_grid  # y栅格数
        self.x_meter_per_grid = self.sampling_map.x_size / self.x_grid  # x每格对应的实际距离(米)
        self.y_meter_per_grid = self.sampling_map.y_size / self.y_grid  # y每格对应的实际距离(米)
        self.x_pixel_per_grid = self.sampling_map.pixel_per_meter * self.x_meter_per_grid  # x每格对应的实际长度(像素)
        self.y_pixel_per_grid = self.sampling_map.pixel_per_meter * self.y_meter_per_grid  # y每格对应的实际长度(像素)
        self.map_flag = [[0 for _ in range(x_grid)] for _ in range(y_grid)]

        self.name4image = self.sampling_map.name4image + 'rasterized'

        self.map_rasterization()
        self.draw_rasterization_map(isShow=False)

    def is_grid_has_obs(self, points: list) -> int:
        for _point in points:
            if self.sampling_map.point_is_in_obs(_point):
                return 1
        '''四个顶点都不在障碍物里面'''

        assert len(points) == 4
        for i in range(4):
            if self.sampling_map.line_is_in_obs(points[i % 4], points[(i + 1) % 4]):
                return 1
        '''四个边都不在障碍物里面'''

        for _obs in self.sampling_map.obs:
            if _obs[0] == 'circle' or _obs[0] == 'ellipse':
                if self.sampling_map.point_is_in_poly(center=None, r=None, points=points, point=_obs[2]):
                    return 1
            else:
                if self.sampling_map.point_is_in_poly(center=None, r=None, points=points, point=[_obs[1][0], _obs[1][1]]):
                    return 1
        '''障碍物不在格子里面'''
        return 0

    def map_rasterization(self):
        for i in range(self.x_grid):
            for j in range(self.y_grid):
                rec = [[i * self.x_meter_per_grid, j * self.y_meter_per_grid],
                       [(i + 1) * self.x_meter_per_grid, j * self.y_meter_per_grid],
                       [(i + 1) * self.x_meter_per_grid, (j + 1) * self.y_meter_per_grid],
                       [i * self.x_meter_per_grid, (j + 1) * self.y_meter_per_grid]]
                self.map_flag[i][j] = self.is_grid_has_obs(rec)

    '''drawing'''

    def map_draw_photo_frame(self):
        cv.rectangle(self.sampling_map.image, (0, 0), (self.sampling_map.width - 1, self.sampling_map.dis2pixel([self.sampling_map.x_size, self.sampling_map.y_size])[1]), Color().White, -1)
        cv.rectangle(self.sampling_map.image, (0, 0), (self.sampling_map.dis2pixel([0., 0.])[0], self.sampling_map.height - 1), Color().White, -1)
        cv.rectangle(self.sampling_map.image, self.sampling_map.dis2pixel([self.sampling_map.x_size, self.sampling_map.y_size]), (self.sampling_map.width - 1, self.sampling_map.height - 1),
                     Color().White, -1)
        cv.rectangle(self.sampling_map.image, self.sampling_map.dis2pixel([0., 0.]), (self.sampling_map.width - 1, self.sampling_map.height - 1), Color().White, -1)

    def draw_rasterization_map(self, isShow=True, isWait=True):
        self.map_draw_gird_rectangle()
        self.map_draw_x_grid()
        self.map_draw_y_grid()
        self.sampling_map.map_draw_start_terminal()
        self.sampling_map.map_draw_obs()
        self.sampling_map.map_draw_photo_frame()
        self.sampling_map.map_draw_boundary()
        self.sampling_map.map_draw_start_terminal()
        if isShow:
            cv.imshow(self.name4image, self.sampling_map.image)
            cv.waitKey(0) if isWait else cv.waitKey(1)
        self.sampling_map.image = self.sampling_map.image_temp.copy()

    def map_draw_gird_rectangle(self):
        for i in range(self.x_grid):
            for j in range(self.y_grid):
                if self.map_flag[i][j] == 1:
                    pt1 = self.grid2pixel(coord_int=[i, j], pos='left-bottom', xoffset=-0, yoffset=0)
                    pt2 = self.grid2pixel(coord_int=[i, j], pos='right-top', xoffset=0, yoffset=0)
                    cv.rectangle(self.sampling_map.image, pt1, pt2, Color().LightGray, -1)

    def grid2pixel(self, coord_int: list, pos: str, xoffset=0, yoffset=0) -> tuple:
        """
        :brief:             to transfer grid in map to pixel in image
        :param coord_int:   coordinate [int, int]
        :param pos:         left-top, left-bottom, right-top. right-bottom
        :param xoffset:     xoffset
        :param yoffset:     yoffset
        :return:            pixel [int, int] (left-bottom)
        :tips:              the direction of offset is the same as that of image rather than real world or grid map
        """
        x = self.sampling_map.x_offset + coord_int[0] * self.x_pixel_per_grid
        y = self.sampling_map.height - self.sampling_map.y_offset - coord_int[1] * self.y_pixel_per_grid  # sef default to left-bottom

        if pos == 'left-bottom':
            return int(x) + xoffset, int(y) + yoffset
        elif pos == 'left-top':
            return int(x) + xoffset, int(y - self.y_pixel_per_grid) + yoffset
        elif pos == 'right-bottom':
            return int(x + self.x_pixel_per_grid) + xoffset, int(y) + yoffset
        elif pos == 'right-top':
            return int(x + self.x_pixel_per_grid) + xoffset, int(y - self.y_pixel_per_grid) + yoffset
        else:
            print('FUNCTION <grid2pixel>--ERROR input')
            return ()

    def map_draw_x_grid(self):
        for i in range(self.y_grid + 1):
            pt1 = self.grid2pixel(coord_int=[0, i], pos='left-bottom')
            pt2 = self.grid2pixel(coord_int=[self.x_grid, i], pos='left-bottom')
            cv.line(self.sampling_map.image, pt1, pt2, Color().Black, 1)

    def map_draw_y_grid(self):
        for i in range(self.x_grid + 1):
            pt1 = self.grid2pixel(coord_int=[i, 0], pos='left-bottom')
            pt2 = self.grid2pixel(coord_int=[i, self.y_grid], pos='left-bottom')
            cv.line(self.sampling_map.image, pt1, pt2, Color().Black, 1)

    '''drawing'''

    def point_in_grid(self, point: list) -> list:
        if self.sampling_map.point_is_out(point):
            return [-1, -1]

        return [int(point[0] / self.x_meter_per_grid), int(point[1] / self.y_meter_per_grid)]

    def is_grid_available(self, grid: list) -> bool:
        return True if self.map_flag[grid[0]][grid[1]] == 0 else False

    '''save the map_cfg file'''

    def map_create_database(self, map_num: int, filePath: str, fileName: str):
        """
        map_num:    number of the maps
        filePath:
        fileName:
        """
        f = open(file=filePath + fileName, mode='w')
        '''First part is the basic message'''
        f.writelines('x_size:' + str(self.sampling_map.x_size) + '\n')
        f.writelines('y_size:' + str(self.sampling_map.y_size) + '\n')
        f.writelines('x_grid:' + str(self.x_grid) + '\n')
        f.writelines('y_grid:' + str(self.y_grid) + '\n')
        '''First part is the basic message'''
        f.writelines('BEGIN' + '\n')
        for i in range(map_num):
            print('num:', i)
            self.sampling_map.set_start([self.sampling_map.x_size / 2, self.sampling_map.y_size / 2])
            self.sampling_map.set_terminal([random.uniform(0.3, self.sampling_map.x_size - 0.3), random.uniform(0.3, self.sampling_map.x_size - 0.3)])
            self.sampling_map.set_random_obstacles(15)
            self.map_rasterization()
            # self.draw_rasterization_map(isShow=True, isWait=False)
            '''Second part is the start-terminal message'''
            f.writelines('num' + str(i) + '\n')
            f.writelines('start:' + str(list(self.sampling_map.start)) + '\n')
            f.writelines('terminal:' + str(list(self.sampling_map.terminal)) + '\n')
            '''Second part is the start-terminal message'''

            '''Third part is the continuous obstacles' message'''
            f.writelines('obs num:' + str(len(self.sampling_map.obs)) + '\n')
            for _obs in self.sampling_map.obs:
                f.writelines(str(_obs).replace('array', '').replace('(', '').replace(')', '') + '\n')
            '''Third part is the continuous obstacles' message'''

            '''Fourth part is the binary grid map'''
            f.writelines(str(self.map_flag).replace(', ', '').replace('[', '').replace(']', '') + '\n')
            '''Fourth part is the binary grid map'''
        f.writelines('END' + '\n')
        f.close()

    '''read the map_cfg file'''

    def map_load_database(self, databaseFile):
        BIG_DATA_BASE = []
        f = open(databaseFile, mode='r')
        ''''检测文件头'''
        assert str(self.sampling_map.x_size) == f.readline().strip('\n')[7:]
        assert str(self.sampling_map.y_size) == f.readline().strip('\n')[7:]
        assert str(self.x_grid) == f.readline().strip('\n')[7:]
        assert str(self.y_grid) == f.readline().strip('\n')[7:]
        assert f.readline().strip('\n') == 'BEGIN'
        ''''检测文件头'''

        line = f.readline().strip('\n')
        while line != 'END':
            DATA = []

            start = f.readline().strip('\n').replace('start:[', '').replace(']', '').replace(' ', '').split(',')
            DATA.append([float(kk) for kk in start])
            terminal = f.readline().strip('\n').replace('terminal:[', '').replace(']', '').replace(' ', '').split(',')
            DATA.append([float(kk) for kk in terminal])

            obsNum = int(f.readline().strip('\n').replace('obs num:', ''))  # obstacles
            DATA.append(obsNum)
            obs_info = []
            while obsNum > 0:
                obs_info.append(self.transfer_str_2_obs_info(f.readline().strip('\n')))  # each obstacle
                obsNum -= 1
            DATA.append(obs_info)
            flag = [[0 for _ in range(self.x_grid)] for _ in range(self.y_grid)]
            binary = f.readline().strip('\n')
            for i in range(self.x_grid*self.y_grid):
                col = i % self.y_grid       # 行数
                row = i // self.y_grid      # 列数
                flag[row][col] = int(binary[i])
            DATA.append(flag)
            BIG_DATA_BASE.append(DATA)
            line = f.readline().strip('\n')
            if line != 'END':
                if int(line[3:]) % 100 == 0:
                    print('...loading env ', int(line[3:]), '...')
        f.close()
        return BIG_DATA_BASE

    @staticmethod
    def transfer_str_2_obs_info(string: str):
        string = string.replace(' ', '').replace("'", '').replace('[', '').replace(']', '').split(',')
        # obs_info = []
        # name_dict = ['circle', 'ellipse', 'triangle', 'rectangle', 'pentagon', 'hexagon', 'heptagon', 'octagon']
        # print(string)
        name = string[0]
        if name == 'circle':
            r, x, y = float(string[1]), float(string[2]), float(string[3])
            obs_info = [name, [r], [x, y]]
        elif name == 'ellipse':
            long, short, theta, x, y = float(string[1]), float(string[2]), float(string[3]), float(string[4]), float(string[5])
            obs_info = [name, [long, short, theta], [x, y]]
        elif name == 'triangle':
            x, y, r = float(string[1]), float(string[2]), float(string[3])
            pts = [[float(string[4 + i * 2]), float(string[5 + i * 2])] for i in range(3)]
            obs_info = [name, [x, y, r], pts]
        elif name == 'rectangle':
            x, y, r = float(string[1]), float(string[2]), float(string[3])
            pts = [[float(string[4 + i * 2]), float(string[5 + i * 2])] for i in range(4)]
            obs_info = [name, [x, y, r], pts]
        elif name == 'pentagon':
            x, y, r = float(string[1]), float(string[2]), float(string[3])
            pts = [[float(string[4 + i * 2]), float(string[5 + i * 2])] for i in range(5)]
            obs_info = [name, [x, y, r], pts]
        elif name == 'hexagon':
            x, y, r = float(string[1]), float(string[2]), float(string[3])
            pts = [[float(string[4 + i * 2]), float(string[5 + i * 2])] for i in range(6)]
            obs_info = [name, [x, y, r], pts]
        elif name == 'heptagon':
            x, y, r = float(string[1]), float(string[2]), float(string[3])
            pts = [[float(string[4 + i * 2]), float(string[5 + i * 2])] for i in range(7)]
            obs_info = [name, [x, y, r], pts]
        elif name == 'octagon':
            x, y, r = float(string[1]), float(string[2]), float(string[3])
            pts = [[float(string[4 + i * 2]), float(string[5 + i * 2])] for i in range(8)]
            obs_info = [name, [x, y, r], pts]
        else:
            assert False
        return obs_info
