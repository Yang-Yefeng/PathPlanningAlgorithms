import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../")

from rasterizedmap import rasterizedmap
import random


if __name__ == '__main__':
    # obs = [
    #     ['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
    #     ['rectangle', [3, 3.5],   [2.0, 5.0, 30.]],
    #     ['rectangle', [4, 1],     [1.5, 5.0, -20.]],
    #     ['pentagon',  [7, 8.5],   [1.0, 180.0]],
    #     ['hexagon',   [8.0, 2],   [1.0, 30.0]],
    #     ['circle',    [6, 6],     [1.0]],
    #     ['ellipse',   [3, 8],     [2.6, 0.6, -20.0]],
    #     ['heptagon',  [4, 6],     [1.0, 0.]],
    #     ['octagon',   [6, 4],     [1.0, 0.]],
    # ]
    # obs = obstacle(obs).get_obs()
    obs = []
    x_size = 5
    y_size = 5
    x_grid_per_meter = 10
    y_grid_per_meter = 10
    r_map = rasterizedmap(width=500,
                          height=500,
                          x_size=x_size,
                          y_size=y_size,
                          image_name='rasteriazedmap',
                          start=None,
                          terminal=None,
                          obs=obs,
                          draw=False,
                          x_grid=x_size * x_grid_per_meter,
                          y_grid=y_size * y_grid_per_meter)  # 生成栅格化地图
    # r_map.set_start([random.uniform(0.15, r_map.x_size - 0.15), random.uniform(0.15, r_map.y_size - 0.15)])
    # r_map.set_start([2.5, 2.5])
    # r_map.set_terminal([random.uniform(0.15, r_map.x_size - 0.15), random.uniform(0.15, r_map.y_size - 0.15)])
    # r_map.set_random_obstacles(20)
    # r_map.map_rasterization()
    # r_map.draw_rasterization_map(isShow=True, isWait=True)
    r_map.map_create_database(map_num=1000, filePath='', fileName='DataBase0.txt')
    # r_map.test4database()
