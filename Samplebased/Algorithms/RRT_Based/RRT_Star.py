import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../../PathPlanningAlgorithms/")

from Samplebased.Algorithms.RRT_Based import *
from Samplebased.Algorithms.RRT_Based.RRT import RRT


class RRT_Star(RRT):
    def __init__(self, width, height, x_size, y_size, image_name, start, terminal, obs, map_file):
        super(RRT_Star, self).__init__(width, height, x_size, y_size, image_name, start, terminal, obs, map_file)

        self.tree_point = []
        self.circle_r = 0.6
        self.cost = dict()                          # record the cost of nodes

        '''initialization'''
        self.tree_point.append(self.start)
        self.cost[tuple(self.start)] = 0.0
        '''initialization'''

    def find_nodes_in_circle(self, point):
        nodes = []
        for _iter in self.tree_point:
            bool1 = (_iter[0] - point[0]) ** 2 + (_iter[1] - point[1]) ** 2 <= self.circle_r ** 2
            bool2 = _iter != self.start
            bool3 = _iter != point
            # bool4 = iter != list(self.parent[tuple(point)])
            if bool1 and bool2 and bool3:
                nodes.append(_iter)
        return nodes

    def rewire(self, new_node):
        """
        :Tips:              为new_nodes执行rewire
                                1. 为每一个new_node选择其parent
                                2. 若其他节点以new_node为父节点时可以缩短代价，则更新
        :param new_node:
        :return:
        """
        nodes_in_circle = self.find_nodes_in_circle(new_node)       # 自动扣除new_node自身和它固有的parent
        '''1. Update parent of the new_node'''
        for node in nodes_in_circle:
            cost0 = self.cost[tuple(new_node)]
            if cost0 > self.cost[tuple(node)] + self.step:
                if self.line_is_in_obs(node, new_node):
                    continue
                self.cost[tuple(new_node)] = self.cost[tuple(node)] + self.step
                self.parent[tuple(new_node)] = tuple(node)
        '''1. Update parent of the new_node'''
        '''2. Rewire'''
        for node in nodes_in_circle:
            cost0 = self.cost[tuple(node)]
            if cost0 > self.cost[tuple(new_node)] + self.step:
                if self.line_is_in_obs(node, new_node):
                    continue
                self.cost[tuple(node)] = self.cost[tuple(new_node)] + self.step
                self.parent[tuple(node)] = tuple(new_node)
        '''2. Rewire'''

    def search_nearest_node_and_tree_generate_with_rewire(self, points):
        new_nodes = []
        for point in points:
            node, _ = self.tree.search_nn(point)                                    # 开始寻找在KDTree中离point最近的节点
            new_node = self.tree_generate_with_start_end_point(node.data, point)    # 新的节点，但是没有做障碍物检测
            if self.point_is_in_obs(new_node):
                continue
            if self.line_is_in_obs(new_node, node.data):
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

    def rrt_star_main(self, is_dynamic_show=False):
        step = 0
        video_record = cv.VideoWriter('../../../somefigures/video/mp4/rrt_star.mp4', cv.VideoWriter_fourcc(*'mp4v'), 120, (self.width, self.height))
        while step <= 10000:
            step += 1
            dir_points = self.create_random_points_in_map(50)
            new_nodes = self.search_nearest_node_and_tree_generate_with_rewire(dir_points)
            for new_node in new_nodes:
                '''draw dynamic map'''
                self.map_draw_obs()
                self.map_draw_photo_frame()
                self.map_draw_boundary()
                self.map_draw_start_terminal()
                cv.line(self.image, self.dis2pixel(new_node), self.dis2pixel(self.parent[tuple(new_node)]), Color().Purple, 1)
                '''draw dynamic map'''
                if (new_node[0] - self.terminal[0]) ** 2 + (new_node[1] - self.terminal[1]) ** 2 <= self.stop_iteration ** 2:
                    self.tree.add(self.terminal)
                    self.parent[tuple(self.terminal)] = tuple(new_node)
                    self.path_find()
                    self.path_draw(self.waypoint, 'rrt_star.png', Color().Orange)
                    for _ in range(120):
                        video_record.write(self.image)
                    video_record.release()
                    return True
            video_record.write(self.image)
            if is_dynamic_show:
                cv.imshow(self.name4image, self.image)
                cv.waitKey(1)
        video_record.release()
        return False


if __name__ == '__main__':
    obstacles1 = [
        ['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
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
    obstacles2 = [
        ['rectangle', [7, 8],   [2.0, 85.0, 0.]],
        ['rectangle', [4, 6],   [3.2, 5.0, 0.]],
        ['rectangle', [3, 3.5], [2.0, 5.0, 0.]],
        ['rectangle', [5, 2], [2.0, 85.0, 0.]]
    ]
    obstacles3 = [
        ['circle', [6, 6], [1.2]],
        ['circle', [3, 3], [1.0]],
        ['circle', [6, 2], [1.0]],
        ['circle', [1, 9], [1.0]],
        ['circle', [4, 7], [1.0]],
        ['circle', [9, 7], [0.4]],
        ['circle', [3, 5], [1.0]],
        ['circle', [8, 5], [0.5]],
        ['circle', [1, 6], [1.0]],
        ['circle', [8, 1], [0.5]],
        ['circle', [8, 8], [0.5]],
        ['circle', [7.5, 7], [0.5]],
    ]
    obstacles4 = [
        ['ellipse', [6, 6], [1.6, 0.6, -20.0]],
        ['ellipse', [3, 3], [1.6, 1.0, -15.0]],
        ['ellipse', [6, 2], [1.6, 0.4, -10.0]],
        ['ellipse', [5, 9], [3.6, 0.8, -5.0]],
        ['ellipse', [4, 7], [1.6, 0.2, 0.0]],
        ['ellipse', [3.5, 5], [3.6, 0.4, -20.0]],
        ['ellipse', [8, 4.6], [3.6, 0.4, 90.0]],
    ]
    obstacles5 = [
        ['rectangle', [3, 1], [2.0, 10.0, 0.]],
        ['rectangle', [2, 3], [2.0, 5.0, 0.]],
        ['rectangle', [5, 5], [3.0, 5.0, 0.]],
        ['rectangle', [4, 6.5], [2.5, 3.0, 0.]],
        ['rectangle', [8, 8], [2.0, 5.0, 0.]],
    ]
    obstacles = obstacles5
    # obstacles = obstacle(obstacles).get_obs()
    rrt_star = RRT_Star(width=400,
                        height=400,
                        x_size=10,
                        y_size=10,
                        image_name='samplingmap',
                        start=[0.5, 0.5],       # 4.5, 8.5
                        terminal=[9.5, 9.5],
                        obs=obstacles,
                        map_file=None)
    if rrt_star.rrt_star_main(is_dynamic_show=True):
        print('Successful')
    else:
        print('Failed')
