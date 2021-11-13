import copy
from Map.Continuous.obstacle import obstacle
import cv2 as cv
from Map.Color.Color import Color
from Samplebased.Algorithms.RRT_Based.RRT_Star import RRT_Star


class RRT_Star_Smart(RRT_Star):
    def __init__(self, width, height, x_size, y_size, image_name, start, terminal, obs, map_file):
        super(RRT_Star_Smart, self).__init__(width, height, x_size, y_size, image_name, start, terminal, obs, map_file)

    def smart_optimize(self):
        self.path_find()
        self.path_draw(self.waypoint, 'rrt_star_smart.png', Color().Orange)
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

    def rrt_star_smart__main(self, is_dynamic_show=False):
        step = 0
        video_record = cv.VideoWriter('../../../somefigures/video/mp4/rrt_star_smart.mp4', cv.VideoWriter_fourcc(*'mp4v'), 300, (self.width, self.height))
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
                    print('Successful, start to optimize...')
                    print('Press any key to continue...')
                    self.smart_optimize()
                    self.path_find()
                    self.path_draw(self.waypoint, 'rrt_star_smart.png', Color().Red)
                    for _ in range(10):
                        video_record.write(self.image)
                    video_record.release()
                    return True
            video_record.write(self.image)
            if is_dynamic_show:
                cv.imshow(self.name4image, self.image)
                cv.waitKey(1)
        print('Failed')
        video_record.release()
        return False


if __name__ == '__main__':
    obstacles = [
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
    # obstacles = [
    #     ['rectangle', [7, 8],   [2.0, 74.0, 0.]],
    #     ['rectangle', [4, 6],   [3.0, 5.0, 0.]],
    #     ['rectangle', [3, 3.5], [2.0, 5.0, 0.]]
    # ]
    obstacles = obstacle(obstacles).get_obs()
    rrt_star_smart = RRT_Star_Smart(width=400,
                                    height=400,
                                    x_size=10,
                                    y_size=10,
                                    image_name='samplingmap',
                                    start=[0.5, 0.5],       #  4.5, 8.5
                                    terminal=[9.5, 9.5],
                                    obs=obstacles,
                                    map_file=None)
    if rrt_star_smart.rrt_star_smart__main(is_dynamic_show=True):
        print('Successful')
    else:
        print('Failed')
