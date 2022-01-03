import numpy as np
import cv2 as cv
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../shenlanmotionplanning/")

from Map.Color.Color import Color


class grid_map:
    def __init__(self,
                 width: int = 400,
                 height: int = 400,
                 x_grid: int = 10,
                 y_grid: int = 10,
                 image_name: str = 'map',
                 start: list = None,
                 terminal: list = None,
                 obs_number: int = 0,
                 map_file=None):
        """
        :param width:       width of the image
        :param height:      height of the image
        :param x_grid:      grid number
        :param y_grid:      grid number
        :param image_name:  image name
        :param start:       start point
        :param terminal:    terminal point
        :param obs_number:  number of obstacles
        :tips:              For simplification, one grid in map IS EQUAL TO one meter in real world,
                            and the only difference between them is data type, int (former) and double (latter)
        """
        self.width = width
        self.height = height
        if map_file is None:
            self.x_grid = x_grid
            self.y_grid = y_grid
            self.start = np.array([0, 0]) if start is None else np.array(start)
            self.terminal = np.array([x_grid - 1, y_grid - 1]) if terminal is None else np.array(terminal)
            self.obs = np.atleast_2d().clear()
            self.obs_num = obs_number
            assert self.obs_num > 0 or self.obs_num < int(self.x_grid * self.y_grid * 0.5)
            self.map_flag = np.zeros((self.x_grid, self.y_grid), dtype=int)
            '''initialization'''
            self.map_flag[self.start[0], self.start[1]] = 3  # start flag
            '''mark the grid: 0 - default, 1 - open, -1 - closed, 2 - obs, 3 - start, 4 - terminal'''
            self.map_set_obs(is_fixed=False, obs=[[1, 15], [2, 15], [3, 15], [4, 15], [5, 15],
                                                  [6, 15], [7, 15], [8, 15], [9, 15], [10, 15],
                                                  [11, 15], [12, 15], [13, 15], [14, 15],
                                                  [14, 14], [14, 13], [14, 12], [14, 11],
                                                  [14, 10], [14, 9], [14, 8], [14, 7],
                                                  [14, 6], [14, 5], [14, 4], [14, 3], [14, 2],
                                                  [15, 2], [16, 2], [17, 2], [18, 2], [19, 2]])  # set obstacles
            # [5, 0], [5, 1], [5, 2], [5, 3], [5, 4], [5, 5], [5, 6], [5, 7], [5, 8]
            '''initialization'''
        else:
            self.x_grid, self.y_grid, self.start, self.terminal, self.obs, self.obs_num, self.map_flag = self.load_map(map_file)
        self.image = np.zeros([self.width, self.height, 3], np.uint8)
        self.image[:, :, 0] = np.ones([self.width, self.height]) * 255
        self.image[:, :, 1] = np.ones([self.width, self.height]) * 255
        self.image[:, :, 2] = np.ones([self.width, self.height]) * 255
        self.name4image = image_name
        self.x_offset = int(self.width / 20)      # leave blank for image
        self.y_offset = int(self.height / 20)
        self.pixel_per_grid = min((self.width - 2 * self.x_offset) / self.x_grid,
                                  (self.height - 2 * self.y_offset) / self.y_grid)
        self.save_map('map.map')

        self.map_draw()  # draw the map

    def is_out(self, coord):
        return min(coord) < 0 or coord[0] >= self.x_grid or coord[1] >= self.y_grid

    def is_occupied(self, coord: list) -> bool:
        """to check if grid is occupied"""
        temp = self.map_flag[coord[0], coord[1]]
        # return temp == -1 or temp == 2
        return temp == -1 or temp == 2 or temp == 3
        # return temp == -1 or temp == 2 or temp == 3 or temp == 4

    def map_set_obs(self, is_fixed: bool, obs: list):
        if is_fixed:
            assert len(obs) > 0 or len(obs) < int(self.x_grid * self.y_grid * 0.5)
            self.obs_num = len(obs)
            for _obs in obs:
                x = max(min(_obs[0], self.x_grid - 1), 0)
                y = max(min(_obs[1], self.y_grid - 1), 0)
                self.map_flag[x, y] = 2
            self.obs = np.atleast_2d(obs)
        else:
            self.obs = np.atleast_2d(self.get_obs_random())
            print('Set obstacles finished')
            # print('Set obstacles:', self.obs)

    def get_obs_random(self) -> list:
        _obs = []
        for i in range(self.obs_num):
            [obs_x, obs_y] = self.start.tolist()
            while self.is_occupied([obs_x, obs_y]) or [obs_x, obs_y] == list(self.terminal):
                obs_x = np.random.randint(low=0, high=self.x_grid, size=1)[0]
                obs_y = np.random.randint(low=0, high=self.y_grid, size=1)[0]
            _obs.append([obs_x, obs_y])
            self.map_flag[obs_x, obs_y] = 2
        return _obs

    """Transformation"""
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
        x = self.x_offset + coord_int[0] * self.pixel_per_grid
        y = self.height - self.y_offset - coord_int[1] * self.pixel_per_grid  # sef default to left-bottom

        if pos == 'left-bottom':
            return int(x) + xoffset, int(y) + yoffset
        elif pos == 'left-top':
            return int(x) + xoffset, int(y - self.pixel_per_grid) + yoffset
        elif pos == 'right-bottom':
            return int(x + self.pixel_per_grid) + xoffset, int(y) + yoffset
        elif pos == 'right-top':
            return int(x + self.pixel_per_grid) + xoffset, int(y - self.pixel_per_grid) + yoffset
        else:
            print('FUNCTION <grid2pixel>--ERROR input')
            return ()
    """Transformation"""

    """Visualization"""
    def map_draw(self):
        self.map_draw_start_terminal()
        self.map_draw_obs()
        self.map_draw_grid()
        cv.imshow(self.name4image, self.image)
        cv.waitKey(0)
        # cv.imwrite(self.name4image + 'jpg', self.image)

    def map_draw_start_terminal(self):
        pt1 = self.grid2pixel(coord_int=list(self.start), pos='left-bottom')
        pt2 = self.grid2pixel(coord_int=list(self.start), pos='right-top')
        center = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
        cv.circle(self.image, center, int(self.pixel_per_grid / 2 - 2), Color().Red, -1)

        pt1 = self.grid2pixel(coord_int=list(self.terminal), pos='left-bottom')
        pt2 = self.grid2pixel(coord_int=list(self.terminal), pos='right-top')
        center = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
        cv.circle(self.image, center, int(self.pixel_per_grid / 2 - 2), Color().Blue, -1)

    def map_draw_obs(self):
        if self.obs_num > 0:
            for _iter in self.obs:
                pt1 = self.grid2pixel(coord_int=list(_iter), pos='left-bottom', xoffset=-0, yoffset=0)
                pt2 = self.grid2pixel(coord_int=list(_iter), pos='right-top', xoffset=0, yoffset=0)
                # center = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
                cv.rectangle(self.image, pt1, pt2, Color().Orange, -1)
                # cv.circle(self.image, center, int(self.pixel_per_grid / 2 - 2), Color().Red, -1)

    def map_draw_grid(self):
        self.map_draw_start_terminal()
        self.map_draw_x_grid()
        self.map_draw_y_grid()

    def map_draw_x_grid(self):
        for i in range(self.y_grid + 1):
            pt1 = self.grid2pixel(coord_int=[0, i], pos='left-bottom')
            pt2 = self.grid2pixel(coord_int=[self.x_grid, i], pos='left-bottom')
            cv.line(self.image, pt1, pt2, Color().Black, 2)

    def map_draw_y_grid(self):
        for i in range(self.x_grid + 1):
            pt1 = self.grid2pixel(coord_int=[i, 0], pos='left-bottom')
            pt2 = self.grid2pixel(coord_int=[i, self.y_grid], pos='left-bottom')
            cv.line(self.image, pt1, pt2, Color().Black, 2)

    def map_draw_visited(self, visited):
        for visit in visited:
            if visit == list(self.start) or visit == list(self.terminal):
                continue
            pt1 = self.grid2pixel(coord_int=list(visit), pos='left-bottom')
            pt2 = self.grid2pixel(coord_int=list(visit), pos='right-top')
            center = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
            cv.circle(self.image, center, int(self.pixel_per_grid / 2 - 2), Color().Yellow, -1)

    def map_draw_path(self, path):
        length = len(path)
        for i in range(length):
            if 0 < i < length - 1:
                pt1 = self.grid2pixel(coord_int=list(path[i]), pos='left-bottom')
                pt2 = self.grid2pixel(coord_int=list(path[i]), pos='right-top')
                center = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
                cv.circle(self.image, center, int(self.pixel_per_grid / 2 - 2), Color().Thistle, -1)
        for i in range(length):
            if i > 0:
                pt1 = self.grid2pixel(coord_int=list(path[i]), pos='left-bottom')
                pt2 = self.grid2pixel(coord_int=list(path[i]), pos='right-top')
                center1 = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
                pt1 = self.grid2pixel(coord_int=list(path[i - 1]), pos='left-bottom')
                pt2 = self.grid2pixel(coord_int=list(path[i - 1]), pos='right-top')
                center2 = int((pt1[0] + pt2[0]) / 2), int((pt1[1] + pt2[1]) / 2)
                cv.line(self.image, center1, center2, Color().Green, 2)
        cv.imshow(self.name4image, self.image)
        cv.waitKey(0)

    """Visualization"""

    """save"""
    def save_map(self, name=None):
        file = open('map.map', 'w') if name is None else open(name, 'w')
        file.write(str(self.map_flag).replace('[', '').replace(']', '').replace(' ', ''))
        file.write('\n')
        file.write(str(self.terminal[0]).replace('[', '').replace(']', '').replace(' ', ''))
        file.write('\n')
        file.write(str(self.terminal[1]).replace('[', '').replace(']', '').replace(' ', ''))
        file.close()

    def save_image(self, name=None):
        cv.imwrite(self.name4image, self.image) if name is None else cv.imwrite(name, self.image)
    """save"""

    @staticmethod
    def load_map(filename: str):
        """
        :tips:              load a map from an existing map file
        :param filename:
        :return:
        """
        with open(filename, 'r') as file:
            split_data = file.read().split('\n')
        terminaly = int(split_data.pop(-1))
        terminalx = int(split_data.pop(-1))
        terminal = np.array([terminalx, terminaly])
        data = []
        for item in split_data:
            new = []
            for i in item:
                new.append(int(i))
            data.append(new)
        map_flag = np.array(data)
        x_grid, y_grid = map_flag.shape
        start = np.squeeze(np.array(np.where(map_flag == 3)).reshape([1, 2]))
        obs = np.array(np.where(map_flag == 2)).T
        obs_num = obs.shape[0]
        return x_grid, y_grid, start, terminal, obs, obs_num, map_flag
