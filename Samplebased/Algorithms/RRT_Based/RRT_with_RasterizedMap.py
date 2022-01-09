from Map.Rasterized.rasterizedmap import rasterizedmap
from Map.Continuous.samplingmap import samplingmap
from Samplebased.Algorithms.RRT_Based.RRT import RRT

width = 400
height = 400
x_size = 10
y_size = 10

if __name__ == '__main__':
    rrt = RRT(width=width,
              height=height,
              x_size=x_size,
              y_size=y_size,
              image_name='test',
              start=None,  # 4.5, 8.5
              terminal=None,
              obs=[],
              map_file=None)
    sample_map = samplingmap(width=width,
                             height=height,
                             x_size=x_size,
                             y_size=y_size,
                             image_name='test',
                             start=None,
                             terminal=None,
                             obs=[],
                             map_file=None,
                             draw=False)  # 生成连续地图
    ras = rasterizedmap(_samplingmap=sample_map, x_grid=40, y_grid=40)
    DATABASE = ras.map_load_database('../../../Map/Rasterized/10X10-40x40-DataBase-StartFixed/' + 'DataBase09.txt')
    for data in DATABASE:
        rrt.autoSetWithDataBase(data)
        rrt.reset()
        if rrt.rrt_main(is_dynamic_show=True):
            print('Successful')
        else:
            print('Failed')
