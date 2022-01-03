import os
import sys
import numpy as np
import copy

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../../shenlanmotionplanning/")

from Searchingbased.AStar.AStar import AStar


def get_f_from_open_list(elem):
    return elem[3]


class JPS(AStar):
    def __init__(self, width: int,
                 height: int,
                 x_grid: int,
                 y_grid: int,
                 image_name: str,
                 start: list,
                 terminal: list,
                 obs_number: int,
                 map_file):
        super(JPS, self).__init__(width, height, x_grid, y_grid, image_name, start, terminal, obs_number, map_file)

    def jps_main(self) -> bool:
        """
        :return:    if the path exists
        """
        while len(self.open_list) > 0:
            '''choose a node from open list'''
            node = self.open_list.pop(0)  # get the node with the lowest f
            self.map_flag[node[0][0], node[0][1]] = -1  # add the node into close list
            self.close_list.append(node[0])
            self.visited.append(node[0])
            # self.path.append(node[0])                           # add the node to final path
            '''choose a node from open list'''

            '''find the neighbors of the node, and add them into open list'''
            self.get_jps_neighbors_by_jumping(node)         # 咔 咔 咔 一跳一跳找邻居
            self.open_list.sort(key=get_f_from_open_list)
            '''find the neighbors of the node, and add them into open list'''

            if node[0][0] == self.terminal[0] and node[0][1] == self.terminal[1]:
                # self.path.append(node[0])
                print('Path finding finished!')
                # print('visited:', self.visited)
                return True

        print('No path find!')
        return False

    def jump_4neighbor(self, node):
        jump_dir_4neighbor = [[1, 0], [0, 1], [-1, 0], [0, -1]]  # 4 邻域
        for _dir in jump_dir_4neighbor:     # left up right down
            jump_temp = copy.copy(node[0])
            flagx, new_nodesx = self.has_force_neighbor_in_x(jump_temp, _dir[0])
            if flagx:
                for new_node in new_nodesx:
                    # print(new_node)
                    self.update_cost(hyper_node1=node, node2=new_node, heuristic_type='Euclidean')
                    '''
                    Dijkstra Euclidean Manhattan H-Infinite Diagonal
                    '''
            flagy, new_nodesy = self.has_force_neighbor_in_y(jump_temp, _dir[1])
            if flagy:
                for new_node in new_nodesy:
                    self.update_cost(hyper_node1=node, node2=new_node, heuristic_type='Euclidean')
                    '''
                    Dijkstra Euclidean Manhattan H-Infinite Diagonal
                    '''

    def jump_pneighbor(self, node):
        jump_dir_pneighbor = [[1, 1], [-1, 1], [-1, -1], [1, -1]]  # P 邻域
        for _dir in jump_dir_pneighbor:
            jump_temp = copy.copy(node[0])
            while True:     # for each direction
                jump_temp = [jump_temp[0] + _dir[0], jump_temp[1] + _dir[1]]
                if self.is_out(jump_temp):      # 没有 out
                    break
                if self.map_flag[jump_temp[0], jump_temp[1]] == 2:  # 这是障碍物，SB
                    break
                if jump_temp == list(self.terminal):                # 如果是终点，啥也不用说，直接OK就完了
                    self.update_cost(hyper_node1=node, node2=jump_temp, heuristic_type='Euclidean')
                    return
                '''if 'jump_temp' is an unexpanded node'''
                # 按照 X 和 Y 方向分别扩散跳跃，每个方向每一次至多有一个forced neighbor
                flagx, _ = self.has_force_neighbor_in_x(jump_temp, _dir[0])
                if flagx:
                    self.update_cost(hyper_node1=node, node2=jump_temp, heuristic_type='Euclidean')
                    '''
                    Dijkstra Euclidean Manhattan H-Infinite Diagonal
                    '''
                flagy, _ = self.has_force_neighbor_in_y(jump_temp, _dir[1])
                if flagy:
                    self.update_cost(hyper_node1=node, node2=jump_temp, heuristic_type='Euclidean')
                    '''
                    Dijkstra Euclidean Manhattan H-Infinite Diagonal
                    '''

    def get_jps_neighbors_by_jumping(self, node: np.ndarray):
        """
        :tips:              choose new open list nodes based on node
        :param node:
        :return:
        """
        self.jump_4neighbor(node)
        self.jump_pneighbor(node)

    def has_force_neighbor_in_grid(self, node, direction):
        """
        :tips:              to judge if it has force neighbor in 3*3 grid
        :param node:
        :param direction:
        :return:
        """
        # has = False
        has_obs = False
        parent = [node[0] - direction[0], node[1] - direction[1]]
        # force_neighbor = []
        if self.is_out(parent):
            parent = None
        waitinglist = [[m, n] for n in [-1, 0, 1] for m in [-1, 0, 1]]
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                now = [node[0] + i, node[1] + j]
                if self.is_out(now) or (i == 0 and j == 0):
                    waitinglist.remove([i, j])
                    continue
                if now == parent:
                    waitinglist.remove([i, j])
                    continue
                bool1 = self.map_flag[now[0], now[1]] == 2
                bool2 = now == list(self.terminal)
                if bool1 or bool2:
                    waitinglist.remove([i, j])
                    has_obs = True
                    continue
        if not has_obs or len(waitinglist) == 0:
            return False
        # 有 障碍物 或者 terminal
        vector2 = np.array([-direction[0], -direction[1]])
        for each_dir in waitinglist:
            vector1 = np.array(each_dir)
            dot = vector1.dot(vector2)
            if parent is None:      # 没有父亲，自己是自己的父亲，遇到可达点，直接加入force neighbor
                return True
            # 直接判断成立的就行
            if dot == -2:
                return True
            elif dot == -1:
                extra = vector1 + vector2
                extra_vector = list(extra)
                bool1 = self.map_flag[node[0] + extra_vector[0], node[1] + extra_vector[1]] == 2
                bool2 = [node[0] + extra_vector[0], node[1] + extra_vector[1]] == list(self.terminal)
                if bool1:
                    return True
                if bool2:
                    continue
            elif dot == 0:
                extra = vector1 + vector2
                extra_vector = list(np.sign(extra))
                bool1 = self.map_flag[node[0] + extra_vector[0], node[1] + extra_vector[1]] == 2
                bool2 = [node[0] + extra_vector[0], node[1] + extra_vector[1]] == list(self.terminal)
                if bool1 or bool2:
                    return True
            else:
                continue
        return False

    def has_force_neighbor_in_x(self, node, offset) -> (bool, list):
        temp = copy.copy(node)
        if offset == 0:
            return False, None
        force_neighbor = []
        while True:
            temp[0] += offset
            if temp[0] < 0 or temp[0] >= self.x_grid:       # 出界停止
                break
            if self.map_flag[temp[0], temp[1]] == 2:        # 遇到障碍物停止
                break
            if temp == list(self.terminal):                 # 碰到终点停止
                force_neighbor.append(copy.copy(temp))
                break
            if self.has_force_neighbor_in_grid(temp, [offset, 0]):  # [offset, 0] is the direction
                force_neighbor.append(copy.copy(temp))
        return len(force_neighbor) > 0, force_neighbor

    def has_force_neighbor_in_y(self, node, offset) -> (bool, list):
        temp = copy.copy(node)
        if offset == 0:
            return False, None
        force_neighbor = []
        while True:
            temp[1] += offset
            if temp[1] < 0 or temp[1] >= self.y_grid:
                break
            if self.map_flag[temp[0], temp[1]] == 2:
                break
            if temp == list(self.terminal):
                force_neighbor.append(copy.copy(temp))
                break
            if self.has_force_neighbor_in_grid(temp, [0, offset]):
                force_neighbor.append(copy.copy(temp))
        return len(force_neighbor) > 0, force_neighbor


if __name__ == '__main__':
    jps = JPS(width=500,
              height=500,
              x_grid=30,
              y_grid=30,
              image_name='JPS',
              start=[0, 0],
              terminal=[29, 26],
              obs_number=30,
              map_file='map.map')   # ‘../../Map/Discrete/JPS_Map.map'
    # jps = JPS(width=300,
    #               height=300,
    #               x_grid=3,
    #               y_grid=3,
    #               image_name='Jump Point Search',
    #               start=[0, 0],
    #               terminal=[2, 2],
    #               obs_number=0)
    # jps = JPS(width=400,
    #           height=400,
    #           x_grid=4,
    #           y_grid=4,
    #           image_name='Jump Point Search',
    #           start=[0, 0],
    #           terminal=[3, 3],
    #           obs_number=4)
    # jps.save_map('../../Map/Discrete/JPS_Map.map')
    if jps.jps_main():
        jps.path_find()
        jps.map_draw_visited(jps.visited)
        jps.map_draw_path(jps.path)
        # name = jps.name4image + str(jps.x_grid) + '_' + str(jps.y_grid) + '_' + str(jps.start) + '_' + str(jps.terminal) + '_' + str(jps.obs_num)
        # print(name)
        jps.save_image(name='jps_1.jpg')
