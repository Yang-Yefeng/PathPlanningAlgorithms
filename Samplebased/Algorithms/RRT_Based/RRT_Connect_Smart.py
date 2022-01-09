import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../../PathPlanningAlgorithms/")

from Samplebased.Algorithms.RRT_Based import *
from Samplebased.Algorithms.RRT_Based.RRT_Connect import RRT_Connect


class RRT_Connect_Smart(RRT_Connect):
    def __init__(self, width, height, x_size, y_size, image_name, start, terminal, obs, map_file):
        super(RRT_Connect_Smart, self).__init__(width, height, x_size, y_size, image_name, start, terminal, obs, map_file)

    def smart_optimize(self):
        s = tuple(self.terminal)
        sterminal = tuple(self.terminal)
        while True:
            ss = self.parent[s]
            if not self.line_is_in_obs(list(sterminal), list(ss)):
                self.parent[sterminal] = ss
                s = ss
            else:
                print('A new line:', s, ss, sterminal)
                sterminal = copy.deepcopy(s)
            if s == self.parent[s]:  # tuple(self.start)
                break

    def rrt_connect_smart_main(self, is_dynamic_show=False):
        step = 0
        video_record = cv.VideoWriter('../../../somefigures/video/mp4/rrt_connect_smart.mp4', cv.VideoWriter_fourcc(*'mp4v'), 120, (self.width, self.height))
        while step < 10000:
            step += 1
            dir_points = self.create_random_points_in_map(50)
            new_node_starts = self.search_nearest_node_and_tree_generate_from_start(dir_points)
            new_node_terminals = self.search_nearest_node_and_tree_generate_from_terminal(dir_points)
            for new_node_start in new_node_starts:
                '''draw dynamic map'''
                self.map_draw_obs()
                self.map_draw_photo_frame()
                self.map_draw_boundary()
                self.map_draw_start_terminal()
                cv.line(self.image, self.dis2pixel(new_node_start), self.dis2pixel(self.parent_start[tuple(new_node_start)]), Color().Purple, 1)
                '''draw dynamic map'''
            for new_node_terminal in new_node_terminals:
                '''draw dynamic map'''
                self.map_draw_obs()
                self.map_draw_photo_frame()
                self.map_draw_boundary()
                self.map_draw_start_terminal()
                cv.line(self.image, self.dis2pixel(new_node_terminal), self.dis2pixel(self.parent_terminal[tuple(new_node_terminal)]), Color().DarkGreen, 1)
                '''draw dynamic map'''
            if is_dynamic_show:
                cv.imshow(self.name4image, self.image)
                cv.waitKey(1)
            video_record.write(self.image)
            if self.two_tree_connect():
                self.path_find()
                print('Successful, start to optimize...')
                print('Press any key to continue...')
                self.path_draw(self.waypoint, 'rrt_connect_smart.png', Color().Orange)
                self.smart_optimize()
                self.path_find2()
                self.path_draw(self.waypoint, 'rrt_connect_smart.png', Color().Red)
                for _ in range(120):
                    video_record.write(self.image)
                video_record.release()
                return True
        video_record.release()
        return False

    def path_find2(self):
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
    # obstacles = obstacle(obstacles).get_obs()
    rrt_connect = RRT_Connect_Smart(width=400,
                                    height=400,
                                    x_size=10,
                                    y_size=10,
                                    image_name='samplingmap',
                                    start=[0.5, 0.5],     # 0.5, 0.5  4.5, 8.5
                                    terminal=[9.5, 9.5],
                                    obs=obstacles,
                                    map_file=None)
    if rrt_connect.rrt_connect_smart_main(is_dynamic_show=True):
        print('Successful')
    else:
        print('Failed')
