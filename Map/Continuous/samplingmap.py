import copy

import numpy as np
import cv2 as cv
import os
import sys
import math

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../shenlanmotionplanning/")

from Map.Color.Color import Color

def sind(theta):
    return math.sin(theta / 180.0 * math.pi)

def cosd(theta):
    return math.cos(theta / 180.0 * math.pi)


class samplingmap:
    def __init__(self,
                 width: int = 400,
                 height: int = 400,
                 x_size: float = 10.,
                 y_size: float = 10.,
                 image_name: str = 'samplingmap',
                 start: list = None,
                 terminal: list = None,
                 obs=None,
                 map_file=None):
        self.width = width
        self.height = height
        if map_file is None:
            self.x_size = x_size
            self.y_size = y_size
            self.name4image = image_name
            # self.start = self.start = np.array([0.5, 0.5]) if start is None else np.array(start)
            self.start = self.start = [0.5, 0.5] if start is None else start
            # self.terminal = np.array([x_size - 0.5, y_size - 0.5]) if terminal is None else np.array(terminal)
            self.terminal = [x_size - 0.5, y_size - 0.5] if terminal is None else terminal
            self.obs = obs
            self.obs_num = 0 if obs is None else len(obs)
        else:
            pass
        self.image = np.zeros([self.width, self.height, 3], np.uint8)
        self.image[:, :, 0] = np.ones([self.width, self.height]) * 255
        self.image[:, :, 1] = np.ones([self.width, self.height]) * 255
        self.image[:, :, 2] = np.ones([self.width, self.height]) * 255
        self.image_white = self.image.copy()        # 纯白图

        self.name4image = image_name
        self.x_offset = int(self.width / 20)  # leave blank for image
        self.y_offset = int(self.height / 20)
        self.pixel_per_meter = min((self.width - 2 * self.x_offset) / self.x_size,
                                   (self.height - 2 * self.y_offset) / self.y_size)

        self.map_draw()
        self.image_temp = self.image.copy()

    def point_is_out(self, point: list):
        return min(point) < 0 or point[0] >= self.x_size or point[1] >= self.y_size

    @staticmethod
    def cross_product(vec1, vec2):
        return vec1[0] * vec2[1] - vec2[0] * vec1[1]

    @staticmethod
    def dis_two_points(point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    @staticmethod
    def point_is_in_circle(center, r, point):
        sub = [center[i] - point[i] for i in [0, 1]]
        return np.linalg.norm(sub) <= r

    @staticmethod
    def point_is_in_ellipse(long, short, rotate_angle, center, point):
        """
        :param long:                长轴
        :param short:               短轴
        :param rotate_angle:        椭圆自身的旋转角度
        :param center:              中心点
        :param point:               待测点
        :return:                    bool
        """
        sub = np.array([point[i] - center[i] for i in [0, 1]])
        trans = np.array([[cosd(-rotate_angle), -sind(-rotate_angle)], [sind(-rotate_angle), cosd(-rotate_angle)]])
        [x, y] = list(np.dot(trans, sub))
        return (x / long) ** 2 + (y / short) ** 2 <= 1

    def point_is_in_poly(self, center, r, points, point):
        if self.point_is_in_circle(center, r, point) is False:
            return False
        '''若在多边形对应的外接圆内，再进行下一步判断'''
        l_pts = len(points)
        res = False
        j = l_pts - 1
        for i in range(l_pts):
            if ((points[i][1] > point[1]) != (points[j][1] > point[1])) and \
                    (point[0] < (points[j][0] - points[i][0]) * (point[1] - points[i][1]) / (points[j][1] - points[i][1]) + points[i][0]):
                res = not res
            j = i
        if res is True:
            return True

    def line_is_in_circle(self, center, r, point1, point2):
        return self.line_is_in_ellipse(r, r, 0, center, point1, point2)

    def line_is_in_ellipse(self, long, short, rotate_angle, center, point1, point2):
        """
        :param long:                长轴
        :param short:               短轴
        :param rotate_angle:        椭圆自身的旋转角度
        :param center:              中心点
        :param point1:              待测点1
        :param point2:              待测点2
        :return:                    bool
        """
        if self.point_is_in_ellipse(long, short, rotate_angle, center, point1):
            return True
        if self.point_is_in_ellipse(long, short, rotate_angle, center, point2):
            return True
        pt1 = [point1[i] - center[i] for i in [0, 1]]
        pt2 = [point2[j] - center[j] for j in [0, 1]]       # 平移至原点

        pptt1 = [pt1[0]*cosd(-rotate_angle) - pt1[1]*sind(-rotate_angle), pt1[0]*sind(-rotate_angle) + pt1[1]*cosd(-rotate_angle)]
        pptt2 = [pt2[0]*cosd(-rotate_angle) - pt2[1]*sind(-rotate_angle), pt2[0]*sind(-rotate_angle) + pt2[1]*cosd(-rotate_angle)]

        if pptt1[0] == pptt2[0]:
            if short ** 2 * (1 - pptt1[0] ** 2 / long ** 2) < 0:
                return False
            else:
                y_cross = math.sqrt(short ** 2 * (1 - pptt1[0] ** 2 / long ** 2))
                if max(pptt1[1], pptt2[1]) >= y_cross >= -y_cross >= min(pptt1[1], pptt2[1]):
                    return True
                else:
                    return False
        else:
            k = (pptt2[1] - pptt1[1]) / (pptt2[0] - pptt1[0])
            b = pptt1[1] - k * pptt1[0]
            ddelta = (long * short) ** 2 * (short ** 2 + long ** 2 * k ** 2 - b ** 2)
            if ddelta < 0:
                # print('Delat is negative..')
                return False
            else:
                x_medium = -(k * b * long ** 2) / (short ** 2 + long ** 2 * k ** 2)
                if max(pptt1[0], pptt2[0]) >= x_medium >= min(pptt1[0], pptt2[0]):
                    # print('Yes')
                    return True
                else:
                    # print('Haha, too short...')
                    return False

    def line_is_in_poly(self, center, r, points, point1, point2):
        if self.point_is_in_poly(center, r, points, point1):
            print('Something wrong happened...')
            return True
        if self.point_is_in_poly(center, r, points, point2):
            print('Something wrong happened...')
            return True
        length = len(points)
        # c = point1
        # d = point2
        for i in range(length):
            a = points[i % length]
            b = points[(i + 1) % length]
            c = copy.deepcopy(point1)
            d = copy.deepcopy(point2)
            '''通过坐标变换将a点变到原点'''
            b = [b[i] - a[i] for i in [0, 1]]
            c = [c[i] - a[i] for i in [0, 1]]
            d = [d[i] - a[i] for i in [0, 1]]
            a = [a[i] - a[i] for i in [0, 1]]
            '''通过坐标变换将a点变到原点'''
            '''通过坐标旋转将b点变到与X重合'''
            l_ab = self.dis_two_points(a, b)     # length of ab
            cos = b[0] / l_ab
            sin = b[1] / l_ab
            bb = [cos * b[0] + sin*b[1], -sin*b[0] + cos*b[1]]
            cc = [cos * c[0] + sin*c[1], -sin*c[0] + cos*c[1]]
            dd = [cos * d[0] + sin*d[1], -sin*d[0] + cos*d[1]]
            '''通过坐标旋转将b点变到与X重合'''
            if cc[1] * dd[1] > 0:
                '''如果变换后的cd纵坐标在x轴的同侧'''
                # return False
                continue
            else:
                '''如果变换后的cd纵坐标在x轴的异侧(包括X轴)'''
                if cc[0] == dd[0]:
                    '''k == inf'''
                    if min(bb) <= cc[0] <= max(bb):
                        return True
                    else:
                        continue
                else:
                    '''k != inf'''
                    k_cd = (dd[1] - cc[1]) / (dd[0] - cc[0])
                    b_cd = cc[1] - k_cd * cc[0]
                    x_cross = -b_cd / k_cd
                    if min(bb) <= x_cross <= max(bb):
                        return True
                    else:
                        continue
        return False

    def line_is_in_obs(self, point1, point2) -> bool:
        for _obs in self.obs:
            if _obs[0] == 'circle':
                if self.line_is_in_circle(_obs[2], _obs[1][0], point1, point2):
                    # print('line_is_in_circle', point1, point2)
                    # print('line_is_in_circle')
                    return True
            elif _obs[0] == 'ellipse':
                if self.line_is_in_ellipse(_obs[1][0], _obs[1][1], _obs[1][2], _obs[2], point1, point2):
                    # print('line_is_in_ellipse', point1, point2)
                    # print('line_is_in_ellipse')
                    return True
            else:
                if self.line_is_in_poly([_obs[1][i] for i in [0, 1]], _obs[1][2], _obs[2], point1, point2):
                    # print('line_is_in_poly', point1, point2)
                    # print('line_is_in_poly')
                    return True
        return False

    def point_is_in_obs(self, point: list) -> bool:
        for _obs in self.obs:
            if _obs[0] == 'circle':
                if self.point_is_in_circle(_obs[2], _obs[1][0], point):
                    return True
                else:
                    continue
            elif _obs[0] == 'ellipse':
                if self.point_is_in_ellipse(_obs[1][0], _obs[1][1], _obs[1][2], _obs[2], point):
                    return True
                else:
                    continue
            else:
                if self.point_is_in_poly([_obs[1][i] for i in [0, 1]], _obs[1][2], _obs[2], point):
                    return True
                else:
                    continue
        return False

    def pixel2dis(self, point):
        x = (point[0] - self.x_offset) / self.pixel_per_meter
        y = (self.height - self.y_offset - point[1]) / self.pixel_per_meter
        return [x, y]

    def dis2pixel(self, coord) -> tuple:
        x = self.x_offset + coord[0] * self.pixel_per_meter
        y = self.height - self.y_offset - coord[1] * self.pixel_per_meter
        return int(x), int(y)

    def length2pixel(self, _l):
        return int(_l * self.pixel_per_meter)

    def test_func_point_is_in_obs_using_opencv_callback(self):
        def callback(event, x, y, flags, param):
            self.image_temp = self.image.copy()
            if event == cv.EVENT_MOUSEMOVE:         # 鼠标左键抬起
                point = self.pixel2dis((x, y))
                cv.circle(self.image_temp, (x, y), 3, Color().DarkMagenta, -1)
                if min(point) <= 0. or point[0] > self.x_size or point[1] > self.y_size:
                    cv.putText(self.image_temp, "OUT", (x + 5, y + 5), cv.FONT_HERSHEY_SIMPLEX,
                               0.7, Color().DarkMagenta, 1, cv.LINE_AA)
                else:
                    cv.putText(self.image_temp, str(self.point_is_in_obs(point)), (x + 5, y + 5), cv.FONT_HERSHEY_SIMPLEX,
                               0.7, Color().DarkMagenta, 1, cv.LINE_AA)
        cv.setMouseCallback(self.name4image, callback)
        while True:
            cv.imshow(self.name4image, self.image_temp)
            if cv.waitKey(1) == ord('q'):
                break
        cv.destroyAllWindows()

    def map_draw_boundary(self):
        cv.rectangle(self.image, self.dis2pixel([0., 0.]), self.dis2pixel([self.x_size, self.y_size]), Color().Black, 2)

    def map_draw_start_terminal(self):
        cv.circle(self.image, self.dis2pixel(self.start), 5, Color().Red, -1)
        cv.circle(self.image, self.dis2pixel(self.terminal), 5, Color().Blue, -1)

    def map_draw_obs(self):
        if self.obs is None:
            print('No obstacles!!')
            return
        for [name, constraints, pts] in self.obs:   # [name, [], [pt1, pt2, pt3]]
            if name == 'circle':
                cv.circle(self.image, self.dis2pixel(pts), self.length2pixel(constraints[0]), Color().DarkGray, -1)
            elif name == 'ellipse':
                cv.ellipse(img=self.image,
                           center=self.dis2pixel(pts),
                           axes=(self.length2pixel(constraints[0]), self.length2pixel(constraints[1])),
                           angle=-constraints[2],
                           startAngle=0.,
                           endAngle=360.,
                           color=Color().DarkGray,
                           thickness=-1)
            else:
                cv.fillConvexPoly(self.image, points=np.array([list(self.dis2pixel(pt)) for pt in pts]), color=Color().DarkGray)

    def map_draw(self):
        self.map_draw_boundary()
        self.map_draw_start_terminal()
        self.map_draw_obs()
        cv.imshow(self.name4image, self.image)
        cv.waitKey(0)

    def path_draw(self, path, name, color):
        pt1 = path.pop()
        pt1_int = self.dis2pixel(pt1)
        while path:
            pt2 = path.pop()
            pt2_int = self.dis2pixel(pt2)
            cv.line(self.image, pt1_int, pt2_int, color, 2)
            # cv.imshow(self.name4image, self.image)
            # cv.waitKey(0)
            pt1 = pt2
            pt1_int = self.dis2pixel(pt1)
        cv.imshow(self.name4image, self.image)
        cv.imwrite('../../../somefigures/figure/' + name, self.image)
        cv.waitKey(0)
        cv.destroyAllWindows()
