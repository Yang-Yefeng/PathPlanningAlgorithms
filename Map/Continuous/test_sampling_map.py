from samplingmap import samplingmap
import random
# from obstacle import obstacle


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
    sample_map = samplingmap(width=500,
                             height=500,
                             x_size=10,
                             y_size=10,
                             image_name='samplingmap',
                             start=[5, 5],
                             terminal=None,
                             obs=obs,
                             map_file=None)
    # sample_map.test_func_point_is_in_obs_using_opencv_callback()
    counter = 0
    for _ in range(20):
        sample_map.set_start([random.uniform(0.3, sample_map.x_size - 0.3), random.uniform(0.3, sample_map.y_size - 0.3)])
        sample_map.set_terminal([random.uniform(0.3, sample_map.x_size), random.uniform(0.3, sample_map.y_size - 0.3)])
        sample_map.image = sample_map.image_temp.copy()
        sample_map.set_random_obstacles(10)
        sample_map.map_draw(isWait=False)
        counter += 1
        if counter % 100 == 0:
            print('...', counter, '...')
