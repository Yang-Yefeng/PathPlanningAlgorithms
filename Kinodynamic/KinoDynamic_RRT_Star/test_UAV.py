from Kinodynamic.KinoDynamic_RRT_Star.uav import uav
import matplotlib.pyplot as plt


if __name__ == '__main__':
    UAV = uav(initP=[0., 0., 0],
              initV=[0., 0., 0],
              initA=[0., 0., 0],
              initJ=[0., 0., 0],
              initS=[0., 0., 0],
              maxV=[4., 4., 2.],
              maxA=[1., 1., 1.],
              maxJ=[10., 10., 6.],
              maxS=None,
              minV=[-4., -4., -1.],
              minA=[-1., -1., -1.],
              minJ=[-10., -10., -6.],
              minS=None,
              flag=1)

    sim_time = 10.
    T = [0.]
    uav_j = [UAV.initJ]
    uav_a = [UAV.initA]
    uav_v = [UAV.initV]
    uav_p = [UAV.initP]
    while UAV.time <= sim_time:
        uav_input = [10., 10., 6.]
        UAV.state_update(uav_input)
        uav_j.append(UAV.J)
        uav_a.append(UAV.A)
        uav_v.append(UAV.V)
        uav_p.append(UAV.P)
        T.append(UAV.time)
        UAV.data_save(filepath='', filename='data.csv', is_save_to_file=False)
    UAV.data_save(filepath='', filename='data.csv', is_save_to_file=True)

    plt.figure(1)
    plt.plot(UAV.T, UAV.Jx, label='Jx')
    plt.plot(UAV.T, UAV.Jy, label='Jy')
    plt.plot(UAV.T, UAV.Jz, label='Jz')
    plt.grid(True)
    plt.legend()
    plt.show()

    plt.figure(2)
    plt.plot(UAV.T, UAV.Ax, label='Ax')
    plt.plot(UAV.T, UAV.Ay, label='Ay')
    plt.plot(UAV.T, UAV.Az, label='Az')
    plt.grid(True)
    plt.legend()
    plt.show()

    plt.figure(3)
    plt.plot(UAV.T, UAV.Vx, label='Vx')
    plt.plot(UAV.T, UAV.Vy, label='Vy')
    plt.plot(UAV.T, UAV.Vz, label='Vz')
    plt.grid(True)
    plt.legend()
    plt.show()

    plt.figure(4)
    plt.plot(UAV.T, UAV.Px, label='Px')
    plt.plot(UAV.T, UAV.Py, label='Py')
    plt.plot(UAV.T, UAV.Pz, label='Pz')
    plt.grid(True)
    plt.legend()
    plt.show()
