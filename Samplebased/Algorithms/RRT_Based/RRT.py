import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../../PathPlanningAlgorithms/")

from Samplebased.Algorithms.RRT_Based import *


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
        self.parent[tuple(self.start)] = tuple(self.start)
        '''initialization'''

    def create_random_points_in_map(self, num):
        """
        :param num:
        :return:        points
        """
        return [[random.uniform(0, self.x_size), random.uniform(0, self.y_size)] for _ in range(num)]

    def tree_generate_with_start_end_point(self, start, end):
        [dx, dy] = [end[i] - start[i] for i in range(2)]
        scale = self.step / math.sqrt(dx ** 2 + dy ** 2)
        x = start[0] + dx * scale
        y = start[1] + dy * scale
        return [x, y]

    def search_nearest_node_and_tree_generate(self, points):
        new_nodes = []
        for point in points:
            node, _ = self.tree.search_nn(point)                                    # 开始寻找在KDTree中离point最近的节点
            new_node = self.tree_generate_with_start_end_point(node.data, point)    # 新的节点，但是没有做障碍物检测
            if self.point_is_in_obs(new_node):
                continue
            if self.line_is_in_obs(new_node, node.data):
                '''There would be bugs for RRT_Smart if no 'line_is_in_obs' detection'''
                continue
            new_nodes.append(new_node)
            self.tree.add(new_node)
            self.parent[tuple(new_node)] = tuple(node.data)
            # nearests.append(node.data)
        return new_nodes

    def rrt_main(self, is_dynamic_show=False):
        step = 0
        video_record = cv.VideoWriter('../../../somefigures/video/mp4/rrt.mp4', cv.VideoWriter_fourcc(*'mp4v'), 120, (self.width, self.height))
        while step <= 10000:
            step += 1
            dir_points = self.create_random_points_in_map(50)
            new_nodes = self.search_nearest_node_and_tree_generate(dir_points)
            for new_node in new_nodes:
                '''draw dynamic map'''
                cv.line(self.image, self.dis2pixel(new_node), self.dis2pixel(self.parent[tuple(new_node)]), Color().Purple, 1)
                '''draw dynamic map'''
                if (new_node[0] - self.terminal[0]) ** 2 + (new_node[1] - self.terminal[1]) ** 2 <= self.stop_iteration ** 2:
                    self.tree.add(self.terminal)
                    self.parent[tuple(self.terminal)] = tuple(new_node)
                    self.path_find()
                    self.path_draw(self.waypoint, 'rrt.png', Color().Orange)
                    for _ in range(120):
                        video_record.write(self.image)
                    video_record.release()
                    return True
            if is_dynamic_show:
                cv.imshow(self.name4image, self.image)
                cv.waitKey(1)
            video_record.write(self.image)
        video_record.release()
        return False

    def path_find(self):
        self.waypoint = [tuple(self.terminal)]
        s = tuple(self.terminal)
        while True:
            s = self.parent[s]
            self.waypoint.append(s)
            if s == self.parent[s]:  # tuple(self.start)
                break


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
    obstacles = obstacles4
    obstacles = obstacle(obstacles).get_obs()
    rrt = RRT(width=400,
              height=400,
              x_size=10,
              y_size=10,
              image_name='samplingmap',
              start=[0.5, 0.5],     # 4.5, 8.5
              terminal=[9.5, 9.5],
              obs=obstacles,
              map_file=None)
    if rrt.rrt_main(is_dynamic_show=True):
        print('Successful')
    else:
        print('Failed')
