from gridmap import grid_map


if __name__ == '__main__':
    # map = grid_map(width=700,
    #                height=700,
    #                x_grid=20,
    #                y_grid=20,
    #                image_name='A-Star',
    #                start=[0, 0],
    #                terminal=[19, 18],
    #                obs_number=100)
    map = grid_map(width=300,
                   height=300,
                   x_grid=10,
                   y_grid=10,
                   image_name='map',
                   start=[0, 0],
                   terminal=[9, 2],
                   obs_number=0,
                   map_file=None)
    map.save_map('JPS_Map.map')
    # map.save_image('map.jpg')
