from model import Model


if __name__ == '__main__':
    uav = Model(model_l=0.3,
                model_w=0.3,
                radius=None,
                A=[
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0]
                ],
                B=[
                    [0, 0],
                    [0, 0],
                    [1, 0],
                    [0, 1]
                ],
                s_dimension=4,
                p_dimension=2,
                u_dimension=2,
                s_init=[0,
                        0,
                        0,
                        0],
                time_step=0.01)

    tf = 3
    # u = [1,
    #      1]
    # uav.state_update_time_interval(tf, u)
    # res = uav.differential_equation(uav.s, u)
    #
    # print(res)
    print(uav.s, uav.time)
    n = int(4.0 / uav.time_step)
    u = []
    for _ in range(200):
        u.append([2, 1])
    uav.state_update(u, 2.0)
    print(uav.s, uav.time)
    uav.state_update(u, 2.0)
    print(uav.s, uav.time)
    uav.state_update(u, 2.0)
    print(uav.s, uav.time)
