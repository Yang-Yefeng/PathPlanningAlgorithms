import math
from Samplebased.Base import KDTree
from Map.Continuous.samplingmap import samplingmap
from Map.Continuous.obstacle import obstacle
import random
import cv2 as cv
from Map.Color.Color import Color


class RRT_Connect(samplingmap):
    def __init__(self, width, height, x_size, y_size, image_name, start, terminal, obs, map_file):
        super(RRT_Connect, self).__init__(width, height, x_size, y_size, image_name, start, terminal, obs, map_file)

        self.tree_start = KDTree.create(dimensions=2)     # create a KDTree with dimension 2
        self.tree_terminal = KDTree.create(dimensions=2)     # create a KDTree with dimension 2

        self.tree_start_point = []
        self.tree_terminal_point = []
        self.step = 0.2                             # each step length robot moves
        self.stop_iteration = 0.2
        self.parent_start = dict()
        self.parent_terminal = dict()
        self.waypoint = []

        '''initialization'''
        self.tree_start.add(self.start)
        self.tree_terminal.add(self.terminal)
        '''initialization'''

    def create_random_points_in_map(self, num):
        """
        :param num:
        :return:        points
        """
        return [[random.uniform(0, self.x_size), random.uniform(0, self.y_size)] for _ in range(num)]

    def search_nearest_node_and_tree_generate_from_start(self, points):
        new_nodes = []
        for point in points:
            node, _ = self.tree_start.search_nn(point)                                    # 开始寻找在KDTree中离point最近的节点
            new_node = self.tree_generate_with_start_end_point(node.data, point)    # 新的节点，但是没有做障碍物检测
            if not self.point_is_in_obs(new_node):
                new_nodes.append(new_node)
                self.tree_start.add(new_node)
                self.tree_start_point.append(new_node)
                self.parent_start[tuple(new_node)] = tuple(node.data)
        return new_nodes

    def search_nearest_node_and_tree_generate_from_terminal(self, points):
        new_nodes = []
        for point in points:
            node, _ = self.tree_terminal.search_nn(point)                                    # 开始寻找在KDTree中离point最近的节点
            new_node = self.tree_generate_with_start_end_point(node.data, point)    # 新的节点，但是没有做障碍物检测
            if not self.point_is_in_obs(new_node):
                new_nodes.append(new_node)
                self.tree_terminal.add(new_node)
                self.tree_terminal_point.append(new_node)
                self.parent_terminal[tuple(new_node)] = tuple(node.data)
        return new_nodes

    def tree_generate_with_start_end_point(self, start, end):
        [dx, dy] = [end[i] - start[i] for i in range(2)]
        scale = self.step / math.sqrt(dx ** 2 + dy ** 2)
        x = start[0] + dx * scale
        y = start[1] + dy * scale
        return [x, y]

    def two_tree_connect(self):
        for i in self.tree_start_point:
            for j in self.tree_terminal_point:
                if (i[0] - j[0]) ** 2 + (i[1] - j[1]) ** 2 < self.step ** 2:
                    return True, i, j
        return False, None, None

    def rrt_connect_main(self, is_dynamic_show=False):
        step = 0
        video_record = cv.VideoWriter('rrt_connect.mp4', cv.VideoWriter_fourcc(*'mp4v'), 60, (self.width, self.height))
        while step < 10000:
            step += 1
            dir_points = self.create_random_points_in_map(5)
            new_node_starts = self.search_nearest_node_and_tree_generate_from_start(dir_points)
            new_node_terminals = self.search_nearest_node_and_tree_generate_from_terminal(dir_points)
            for new_node_start in new_node_starts:
                '''draw dynamic map'''
                cv.line(self.image, self.dis2pixel(new_node_start), self.dis2pixel(self.parent_start[tuple(new_node_start)]), Color().Purple, 1)
                '''draw dynamic map'''
            for new_node_terminal in new_node_terminals:
                '''draw dynamic map'''
                cv.line(self.image, self.dis2pixel(new_node_terminal), self.dis2pixel(self.parent_terminal[tuple(new_node_terminal)]), Color().DarkGreen, 1)
                '''draw dynamic map'''
                video_record.write(self.image)
            if is_dynamic_show:
                cv.imshow(self.name4image, self.image)
                cv.waitKey(1)
            connect, pt_connect1, pt_connect2 = self.two_tree_connect()
            if connect:
                print('Successful')
                # cv.waitKey(0)
                video_record.release()
                return connect, pt_connect1, pt_connect2
        print('Failed')
        return False, None, None

    def path_find(self, _pt1, _pt2):
        s = tuple(_pt1)
        while True:
            ss = self.parent_start[s]
            sss = ss
            self.parent_terminal[sss] = s
            s = ss
            if ss == tuple(self.start):
                break
        self.parent_terminal[tuple(_pt1)] = tuple(_pt2)

        self.waypoint = [tuple(self.start)]
        s = tuple(self.start)
        while True:
            s = self.parent_terminal[s]
            self.waypoint.append(s)
            if s == tuple(self.terminal):
                break
        print('Path find finished!')


if __name__ == '__main__':
    obstacles = [['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
                 ['rectangle', [3, 3.5],   [2.0, 5.0, 0.]],
                 ['rectangle', [4, 1],     [1.5, 5.0, -20.]],
                 ['pentagon',  [7, 8.5],   [1.0, 180.0]],
                 ['hexagon',   [8.0, 2],   [1.0, 30.0]],
                 ['triangle',  [8.0, 5],   [1.0, 40.0, 20.0]],
                 ['hexagon',   [5.5, 2],   [0.5, 0.0]],
                 ['circle',    [6, 6],     [1.0]],
                 ['ellipse',   [3, 8],     [2.6, 0.6, -20.0]],
                 ['pentagon',  [3.4, 6.0], [0.6, 50]],
                 ['pentagon',  [8.7, 6.4], [0.8, 108]],
                 ['ellipse',   [1.0, 2.5], [0.8, 0.6, 60.0]],
                 ['pentagon',  [6.5, 4.2], [0.46, 25.0]]]
    obstacles = obstacle(obstacles).get_obs()
    rrt_connect = RRT_Connect(width=400,
                              height=400,
                              x_size=10,
                              y_size=10,
                              image_name='samplingmap',
                              start=[0.5, 0.5],
                              terminal=[9.5, 9.5],
                              obs=obstacles,
                              map_file=None)
    is_path, pt1, pt2 = rrt_connect.rrt_connect_main(is_dynamic_show=True)
    if is_path:
        rrt_connect.path_find(pt1, pt2)
        rrt_connect.path_draw(rrt_connect.waypoint)
