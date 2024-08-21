from samplingmap import samplingmap
import random
import cv2 as cv
# from obstacle import obstacle


if __name__ == '__main__':
    # obs = [['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
    #        ['rectangle', [3, 3.5],   [2.0, 5.0, 0.]],
    #        ['rectangle', [3, 1.5],     [1.5, 15.0, -20.]],
    #        ['pentagon',  [7, 8.5],   [1.0, 180.0]],
    #        ['hexagon',   [8.0, 2],   [1.0, 30.0]],
    #        ['octagon',  [3.0, 9],   [1.0, 30.0]],
    #        ['hexagon',   [5.5, 2],   [0.5, 0.0]],
    #        ['heptagon',    [6, 6],     [1.0, 45]],
    #        ['pentagon',  [3.4, 6.0], [0.6, 50]],
    #        ['pentagon',  [8.7, 6.4], [0.8, 108]]]
    obs = [
        ['triangle',  [1.5, 5],   [1.0, 60.0, 0.0]],
        ['rectangle', [3, 3.5],   [2.0, 5.0, 30.]],
        ['rectangle', [4, 1],     [1.5, 5.0, -20.]],
        ['pentagon',  [7, 8.5],   [1.0, 180.0]],
        ['hexagon',   [8.0, 2],   [1.0, 30.0]],
        ['circle',    [6, 6],     [1.0]],
        ['ellipse',   [3, 8],     [2.6, 0.6, -20.0]],
        ['heptagon',  [4, 6],     [1.0, 0.]],
        ['octagon',   [6, 4],     [1.0, 0.]],
    ]
    # obs = obstacle(obs).get_obs()
    # obs = [
    #        ['rectangle', [4.9, 8.1],   [1.7, 9.0, -45]],
    # ]
    sample_map = samplingmap(width=500,
                             height=500,
                             x_size=11,
                             y_size=11,
                             image_name='samplingmap',
                             start=[5, 5],
                             terminal=None,
                             obs=obs)
    # s = str(sample_map.obs[0][2]).replace('array(', '').replace(')', '').replace(' ', '')
    for _obs in sample_map.obs:
        s = str(_obs).replace('array(', '').replace(')', '')
        print(s)
    # cv.imwrite('sampling_map.png', sample_map.image)
    # print(s)
    # for i in range(10):
    #     sample_map.map_create_continuous_database(map_num=1000, filePath='./DataBase-AllCircle/', fileName='database' + str(i) + '.txt')
    # counter = 0
    # for _ in range(1000):
    #     # sample_map.set_start([sample_map.x_size / 2, sample_map.y_size / 2])
    #     # sample_map.set_start([random.uniform(0.5, sample_map.x_size - 0.5), random.uniform(0.5, sample_map.x_size - 0.5)])
    #     sample_map.set_terminal([random.uniform(0, sample_map.x_size), random.uniform(0, sample_map.x_size)])
    #     sample_map.image = sample_map.image_temp.copy()
    #     sample_map.set_random_obstacles(10)
    #     sample_map.map_draw(isWait=False)
    #     counter += 1
    #     if counter % 100 == 0:
    #         print('...', counter, '...')
