import math

from Samplebased.Base import KDTree
from Map.Continuous.samplingmap import samplingmap
from Map.Continuous.obstacle import obstacle
import random
import cv2 as cv
from Map.Color.Color import Color


class RRT(samplingmap):
    def __init__(self, width, height, x_size, y_size, image_name, start, terminal, obs, map_file):
        super(RRT, self).__init__(width, height, x_size, y_size, image_name, start, terminal, obs, map_file)

        self.tree = KDTree.create(dimensions=2)     # create a KDTree with dimension 2
        self.step = 0.2                             # each step length robot moves
        self.stop_iteration = 0.2
        self.parent = dict()
        self.waypoint = []

        '''initialization'''
        self.tree.add(self.start)
        '''initialization'''

    def create_random_points_in_map(self, num):
        """
        :param num:
        :return:        points
        """
        return [[random.uniform(0, self.x_size), random.uniform(0, self.y_size)] for _ in range(num)]

    def search_nearest_node_and_tree_generate(self, points):
        new_nodes = []
        for point in points:
            node, _ = self.tree.search_nn(point)                                    # 开始寻找在KDTree中离point最近的节点
            new_node = self.tree_generate_with_start_end_point(node.data, point)    # 新的节点，但是没有做障碍物检测
            if not self.point_is_in_obs(new_node):
                new_nodes.append(new_node)
                self.tree.add(new_node)
                self.parent[tuple(new_node)] = tuple(node.data)
            # nearests.append(node.data)
        return new_nodes

    def tree_generate_with_start_end_point(self, start, end):
        [dx, dy] = [end[i] - start[i] for i in range(2)]
        scale = self.step / math.sqrt(dx ** 2 + dy ** 2)
        x = start[0] + dx * scale
        y = start[1] + dy * scale
        return [x, y]

    def rrt_main(self, is_dynamic_show=False):
        step = 0
        while step <= 10000:
            step += 1
            dir_points = self.create_random_points_in_map(1)
            now_nodes = self.search_nearest_node_and_tree_generate(dir_points)
            for new_node in now_nodes:
                '''draw dynamic map'''
                new_node_parent = self.parent[tuple(new_node)]
                new_node_int = self.dis2pixel(new_node)
                new_node_parent_int = self.dis2pixel(new_node_parent)
                cv.line(self.image, new_node_int, new_node_parent_int, Color().Purple, 1)
                cv.imshow(self.name4image, self.image)
                cv.waitKey(1)
                '''draw dynamic map'''
                if (new_node[0] - self.terminal[0]) ** 2 + (new_node[1] - self.terminal[1]) ** 2 <= self.stop_iteration ** 2:
                    self.tree.add(self.terminal)
                    self.parent[tuple(self.terminal)] = tuple(new_node)
                    print('Successful')
                    return True
        print('Failed')
        return False

    def path_find(self):
        self.waypoint = [tuple(self.terminal)]
        s = tuple(self.terminal)
        while True:
            s = self.parent[s]
            self.waypoint.append(s)
            if s == tuple(self.start):
                break


if __name__ == '__main__':
    obs = [['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
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
    obs = obstacle(obs).get_obs()
    rrt = RRT(width=400,
              height=400,
              x_size=10,
              y_size=10,
              image_name='samplingmap',
              start=[0.5, 0.5],
              terminal=[9.5, 9.5],
              obs=obs,
              map_file=None)
    if rrt.rrt_main(is_dynamic_show=True):
        rrt.path_find()
        rrt.path_draw(rrt.waypoint)
