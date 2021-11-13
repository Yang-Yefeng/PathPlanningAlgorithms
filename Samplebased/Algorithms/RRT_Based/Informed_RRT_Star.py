import copy
import math

from Map.Continuous.obstacle import obstacle
import cv2 as cv
from Map.Color.Color import Color
from Samplebased.Algorithms.RRT_Based.RRT_Star import RRT_Star
import numpy as np
from Samplebased.Base import KDTree


class RRT_Informed_Star(RRT_Star):
    def __init__(self, width, height, x_size, y_size, image_name, start, terminal, obs, map_file):
        super(RRT_Informed_Star, self).__init__(width, height, x_size, y_size, image_name, start, terminal, obs, map_file)

        self.e1 = copy.deepcopy(self.start)             # 第一个焦点
        self.e2 = copy.deepcopy(self.terminal)          # 第二个焦点
        self.center = [(self.e1[0] + self.e2[0]) / 2, (self.e1[1] + self.e2[1]) / 2]
        self.rotate_angle = math.degrees(math.acos((max(self.e1[1], self.e2[1]) - min(self.e1[1], self.e2[1])) /
                                                   math.sqrt((self.e2[0] - self.e1[0]) ** 2 + (self.e2[1] - self.e1[1]) ** 2)))
        self.c = math.sqrt((self.e2[0] - self.e1[0]) ** 2 + (self.e2[1] - self.e1[1]) ** 2) / 2      # 焦距
        self.a = math.inf
        self.b = math.sqrt(self.a ** 2 - self.c ** 2)
        self.path_length = math.inf

    def reset(self):
        self.tree = KDTree.create(dimensions=2)     # create a KDTree with dimension 2
        KDTree.visualize(self.tree)
        self.parent.clear()
        self.parent = dict()
        self.waypoint.clear()
        self.waypoint = []

        '''initialization'''
        self.tree.add(self.start)
        self.parent[tuple(self.start)] = tuple(self.start)
        '''initialization'''

        self.tree_point.clear()
        self.tree_point = []
        self.cost.clear()
        self.cost = dict()                          # record the cost of nodes

        '''initialization'''
        self.tree_point.append(self.start)
        self.cost[tuple(self.start)] = 0.0
        '''initialization'''

    def calculate_path_length(self):
        way = copy.deepcopy(self.waypoint)
        self.path_length = 0
        s = copy.deepcopy(way.pop())
        while way:
            ss = copy.deepcopy(way.pop())
            self.path_length += self.dis_two_points(s, ss)
            s = copy.deepcopy(ss)

    def search_nearest_node_and_tree_generate_with_rewire_in_ellipse(self, points):
        new_nodes = []
        for point in points:
            node, _ = self.tree.search_nn(point)                                    # 开始寻找在KDTree中离point最近的节点
            new_node = self.tree_generate_with_start_end_point(node.data, point)    # 新的节点，但是没有做障碍物检测
            if self.point_is_in_obs(new_node):
                continue
            if self.line_is_in_obs(new_node, node.data):
                continue
            heiehi = self.point_is_in_ellipse(self.a, self.b, self.rotate_angle, self.center, new_node)
            if not heiehi:    # 所有点都必须在椭圆中
                continue
            new_nodes.append(new_node)
            self.tree.add(new_node)
            self.tree_point.append(new_node)
            self.parent[tuple(new_node)] = tuple(node.data)                             # 设置parent，但是并没有结束，还需要rewire
            self.cost[tuple(new_node)] = self.step + self.cost[tuple(node.data)]        # 设置cost
            '''rewire'''
            self.rewire(new_node)
            '''rewire'''
        return new_nodes

    def smart_optimize(self):
        self.path_find()
        self.path_draw(self.waypoint, 'rrt_smart.png', Color().Orange)
        s = tuple(self.terminal)
        sterminal = tuple(self.terminal)
        while True:
            ss = self.parent[s]
            if not self.line_is_in_obs(list(sterminal), list(ss)):
                self.parent[sterminal] = ss
                s = ss
            else:
                # print('拐了', s, ss, sterminal)
                sterminal = copy.deepcopy(s)
                # break
            if s == self.parent[s]:  # tuple(self.start)
                break

    def rrt_star_main_once(self, is_dynamic_show=True, video_record=None):
        step = 0
        while step <= 10000:
            step += 1
            dir_points = self.create_random_points_in_map(50)
            new_nodes = self.search_nearest_node_and_tree_generate_with_rewire(dir_points)
            for new_node in new_nodes:
                '''draw dynamic map'''
                cv.line(self.image, self.dis2pixel(new_node), self.dis2pixel(self.parent[tuple(new_node)]), Color().Purple, 1)
                '''draw dynamic map'''
                if (new_node[0] - self.terminal[0]) ** 2 + (new_node[1] - self.terminal[1]) ** 2 <= self.stop_iteration ** 2:
                    self.tree.add(self.terminal)
                    self.parent[tuple(self.terminal)] = tuple(new_node)
                    print('initial search finish...press any key to continue')
                    self.smart_optimize()
                    self.path_find()
                    self.calculate_path_length()
                    # self.path_draw(self.waypoint, 'rrt_star.png', Color().Orange)
                    for _ in range(10):
                        video_record.write(self.image)
                    return True, video_record
            video_record.write(self.image)
            if is_dynamic_show:
                cv.imshow(self.name4image, self.image)
                cv.waitKey(1)
        return False, video_record

    def informed_rrt_star_main(self, is_dynamic_show=False):
        video_record = cv.VideoWriter('../../../somefigures/video/mp4/informed_rrt_star.mp4', cv.VideoWriter_fourcc(*'mp4v'), 300, (self.width, self.height))
        is_success, video_record = self.rrt_star_main_once(is_dynamic_show, video_record)
        '''Inform'''
        if not is_success:
            return
        while True:
            print('waiting...')
            if cv.waitKey(0) == 27:
                break
            self.reset()
            sub_step = 0
            end = False
            self.a = self.path_length / 2
            self.b = math.sqrt(self.a ** 2 - self.c ** 2)
            '''焦点，焦距不变'''
            '''重新绘图'''
            self.image[:, :, 0] = np.ones([self.width, self.height]) * 255
            self.image[:, :, 1] = np.ones([self.width, self.height]) * 255
            self.image[:, :, 2] = np.ones([self.width, self.height]) * 255
            cv.ellipse(img=self.image,
                       center=self.dis2pixel(self.center),
                       axes=(self.length2pixel(self.a), self.length2pixel(self.b)),
                       angle=-self.rotate_angle,
                       startAngle=0.,
                       endAngle=360.,
                       color=Color().DarkMagenta,
                       thickness=2)
            self.map_draw()
            '''重新绘图'''
            '''开始inform'''
            while sub_step < 1000:
                sub_step += 1
                dir_points = self.create_random_points_in_map(20)
                new_nodes = self.search_nearest_node_and_tree_generate_with_rewire_in_ellipse(dir_points)
                for new_node in new_nodes:
                    '''draw dynamic map'''
                    cv.line(self.image, self.dis2pixel(new_node), self.dis2pixel(self.parent[tuple(new_node)]), Color().Purple, 1)
                    '''draw dynamic map'''
                    if (new_node[0] - self.terminal[0]) ** 2 + (new_node[1] - self.terminal[1]) ** 2 <= self.stop_iteration ** 2:
                        self.tree.add(self.terminal)
                        self.parent[tuple(self.terminal)] = tuple(new_node)
                        print('initial search finish...press any key to continue')
                        self.smart_optimize()
                        self.path_find()
                        self.calculate_path_length()
                        self.path_draw(self.waypoint, 'rrt_star.png', Color().Orange)
                        for _ in range(300):
                            video_record.write(self.image)
                        end = True
                        break
                video_record.write(self.image)
                cv.imshow(self.name4image, self.image)
                cv.waitKey(1)
                if end:
                    break
            if sub_step == 1000:
                print('No shorter path, exit...')
                break
        video_record.release()
        return False


if __name__ == '__main__':
    # obstacles = [
    #         ['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
    #         # ['rectangle', [3, 3.5],   [2.0, 5.0, 0.]],
    #         ['rectangle', [4, 1],     [1.5, 5.0, -20.]],
    #         # ['pentagon',  [7, 8.5],   [1.0, 180.0]],
    #         ['hexagon',   [8.0, 2],   [1.0, 30.0]],
    #         # ['triangle',  [8.0, 5],   [1.0, 40.0, 20.0]],
    #         ['hexagon',   [5.5, 2],   [0.5, 0.0]],
    #         ['circle',    [6, 6],     [1.0]],
    #         # ['ellipse',   [3, 8],     [2.6, 0.6, -20.0]],
    #         ['pentagon',  [3.4, 6.0], [0.6, 50]],
    #         ['pentagon',  [8.7, 6.4], [0.8, 108]],
    #         # ['ellipse',   [1.0, 2.5], [0.8, 0.6, 60.0]],
    #         ['pentagon',  [6.5, 4.2], [0.46, 25.0]]
    # ]
    obstacles = [
        ['rectangle', [3, 5],   [2.0, 85.0, 45.]],
        ['rectangle', [4.5, 6],   [2.0, 85.0, 0.]],
        ['rectangle', [6, 3], [2.0, 85.0, 0.]],
        ['rectangle', [4.5, 8], [2.0, 5.0, 0.]]
    ]
    obstacles = obstacle(obstacles).get_obs()
    informed_rrt_star = RRT_Informed_Star(width=400,
                                          height=400,
                                          x_size=10,
                                          y_size=10,
                                          image_name='samplingmap',
                                          start=[3, 6],       # 4.5, 8.5
                                          terminal=[6, 6],
                                          obs=obstacles,
                                          map_file=None)
    if informed_rrt_star.informed_rrt_star_main(is_dynamic_show=True):
        print('Successful')
    else:
        print('Failed')
