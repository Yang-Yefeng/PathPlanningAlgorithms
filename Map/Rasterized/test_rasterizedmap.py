from rasterizedmap import rasterizedmap
from Map.Continuous.samplingmap import samplingmap
# from Map.Continuous.obstacle import obstacle


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
                             start=[0.5, 0.5],
                             terminal=[9.5, 9.5],
                             obs=obs,
                             map_file=None,
                             draw=True)     # 生成连续地图

    r_map = rasterizedmap(_samplingmap=sample_map, x_grid=40, y_grid=40)        # 生成栅格化地图
    # print(r_map.point_in_grid([2.2, 2.2]))
