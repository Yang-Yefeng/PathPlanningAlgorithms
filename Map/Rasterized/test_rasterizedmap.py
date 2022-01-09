from rasterizedmap import rasterizedmap
from Map.Continuous.samplingmap import samplingmap
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
    sample_map = samplingmap(width=500,
                             height=500,
                             x_size=10,
                             y_size=10,
                             image_name='samplingMap',
                             start=[5.0, 5.0],
                             terminal=None,
                             obs=obs,
                             map_file=None,
                             draw=False)     # 生成连续地图
    r_map = rasterizedmap(_samplingmap=sample_map, x_grid=40, y_grid=40)  # 生成栅格化地图
    # r_map.map_create_database(map_num=1000, filePath='', fileName='DataBase01.txt')
    # r_map.map_create_database(map_num=1000, filePath='', fileName='DataBase02.txt')
    # r_map.map_create_database(map_num=1000, filePath='', fileName='DataBase03.txt')
    # r_map.map_create_database(map_num=1000, filePath='', fileName='DataBase04.txt')
    # r_map.map_create_database(map_num=1000, filePath='', fileName='DataBase05.txt')
    DataBase = []
    print('Start Loading DataBase1')
    DataBase.append(r_map.map_load_database('10X10-40x40-DataBase/DataBase01.txt'))
    print('Finish Loading DataBase1')
    print('Start Loading DataBase1')
    DataBase.append(r_map.map_load_database('10X10-40x40-DataBase/DataBase02.txt'))
    print('Finish Loading DataBase1')
    print('Start Loading DataBase1')
    DataBase.append(r_map.map_load_database('10X10-40x40-DataBase/DataBase03.txt'))
    print('Finish Loading DataBase1')
    print('Start Loading DataBase1')
    DataBase.append(r_map.map_load_database('10X10-40x40-DataBase/DataBase04.txt'))
    print('Finish Loading DataBase1')
    print('Start Loading DataBase1')
    DataBase.append(r_map.map_load_database('10X10-40x40-DataBase/DataBase05.txt'))
    print('Finish Loading DataBase1')
    for database in DataBase:
        print('new')
        for data in database:
            r_map.sampling_map.start = data[0]
            r_map.sampling_map.terminal = data[1]
            r_map.sampling_map.obs = data[3]
            r_map.map_flag = data[4]
            r_map.draw_rasterization_map(isShow=True, isWait=False)
