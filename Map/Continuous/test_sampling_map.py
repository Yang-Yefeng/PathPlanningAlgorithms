from samplingmap import samplingmap
from obstacle import obstacle


if __name__ == '__main__':
    # obs = [['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
    #        ['rectangle', [3, 3.5],   [2.0, 5.0, 0.]],
    #        ['rectangle', [4, 1],     [1.5, 5.0, -20.]],
    #        ['pentagon',  [7, 8.5],   [1.0, 180.0]],
    #        ['hexagon',   [8.0, 2],   [1.0, 30.0]],
    #        ['triangle',  [8.0, 5],   [1.0, 30.0, 20.0]],
    #        ['hexagon',   [5.5, 2],   [0.5, 0.0]],
    #        ['circle',    [6, 6],     [1.0]],
    #        ['ellipse',   [3, 8],     [2.6, 0.6, -20.0]],
    #        ['pentagon',  [3.4, 6.0], [0.6, 50]],
    #        ['pentagon',  [8.7, 6.4], [0.8, 108]]]
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
    sample_map = samplingmap(width=400,
                             height=400,
                             x_size=10,
                             y_size=10,
                             image_name='samplingmap',
                             start=[0.5, 0.5],
                             terminal=[9.5, 9.5],
                             obs=obs,
                             map_file=None)
    # sample_map.test_func_point_is_in_obs_using_opencv_callback()
