import copy
from Map.Continuous.obstacle import obstacle
import cv2 as cv
from Map.Color.Color import Color
from Samplebased.Algorithms.RRT_Based.RRT import RRT


class RRT_Smart(RRT):
    def __init__(self, width, height, x_size, y_size, image_name, start, terminal, obstacles, map_file):
        super(RRT_Smart, self).__init__(width, height, x_size, y_size, image_name, start, terminal, obstacles, map_file)

    def smart_optimize(self):
        self.path_find()
        self.path_draw(self.waypoint, 'rrt_with_no_smart.png')
        s = tuple(self.terminal)
        sterminal = tuple(self.terminal)
        while True:
            ss = self.parent[s]
            if not self.line_is_in_obs(list(sterminal), list(ss)):
                self.parent[sterminal] = ss
                s = ss
            else:
                print('拐了', s, ss, sterminal)
                sterminal = copy.deepcopy(s)
                # break
            if s == self.parent[s]:  # tuple(self.start)
                break

    def rrt_smart_main(self, is_dynamic_show=False):
        step = 0
        while step <= 10000:
            step += 1
            dir_points = self.create_random_points_in_map(5)
            new_nodes = self.search_nearest_node_and_tree_generate(dir_points)
            for new_node in new_nodes:
                if is_dynamic_show:
                    '''draw dynamic map'''
                    cv.line(self.image, self.dis2pixel(new_node), self.dis2pixel(self.parent[tuple(new_node)]), Color().Purple, 1)
                    cv.imshow(self.name4image, self.image)
                    cv.waitKey(1)
                    '''draw dynamic map'''
                if (new_node[0] - self.terminal[0]) ** 2 + (new_node[1] - self.terminal[1]) ** 2 <= self.stop_iteration ** 2:
                    self.tree.add(self.terminal)
                    self.parent[tuple(self.terminal)] = tuple(new_node)
                    print('Successful, start to optimize...')
                    self.smart_optimize()
                    return True
        print('Failed')
        return False


if __name__ == '__main__':
    obs = [
        ['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
        ['rectangle', [3, 3.5],   [2.0, 5.0, 0.]],
        ['rectangle', [4, 1],     [1.5, 5.0, -20.]],
        ['pentagon',  [7, 8.5],   [1.0, 180.0]],
        ['hexagon',   [8.0, 2],   [1.0, 30.0]],
        ['triangle',  [8.0, 5],   [1.0, 30.0, 20.0]],
        ['hexagon',   [5.5, 2],   [0.5, 0.0]],
        ['circle',    [6, 5],     [1.0]],
        ['ellipse',   [2, 8],     [2, 0.6, -20.0]],
        ['pentagon',  [3.4, 6.0], [0.6, 50]],
        ['pentagon',  [8.7, 6.4], [0.8, 108]]
    ]
    obs = obstacle(obs).get_obs()
    rrt_smart = RRT_Smart(width=400,
                          height=400,
                          x_size=10,
                          y_size=10,
                          image_name='samplingmap',
                          start=[0.5, 0.5],
                          terminal=[9.5, 9.5],
                          obstacles=obs,
                          map_file=None)
    if rrt_smart.rrt_smart_main(is_dynamic_show=True):
        rrt_smart.path_find()
        rrt_smart.path_draw(rrt_smart.waypoint, 'rrt_smart.png')
