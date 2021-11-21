from Kinodynamic.model.model import Model
from Map import *


def sind(theta):
    return math.sin(theta / 180.0 * math.pi)

def cosd(theta):
    return math.cos(theta / 180.0 * math.pi)


class KinoDynamic(Model, samplingmap):
    def __init__(self, _model_parameters, _map_parameters):
        Model.__init__(self,
                       _model_parameters['model_l'],
                       _model_parameters['model_w'],
                       _model_parameters['radius'],
                       _model_parameters['A'],
                       _model_parameters['B'],
                       _model_parameters['s_dimension'],
                       _model_parameters['p_dimension'],
                       _model_parameters['u_dimension'],
                       _model_parameters['s_init'],
                       _model_parameters['time_step'])
        samplingmap.__init__(self,
                             _map_parameters['width'],
                             _map_parameters['height'],
                             _map_parameters['x_size'],
                             _map_parameters['y_size'],
                             _map_parameters['image_name'],
                             _map_parameters['start'],
                             _map_parameters['terminal'],
                             _map_parameters['obs'],
                             _map_parameters['map_file'])

    def draw_model(self, origination=None, position=None):
        origination = origination if origination else 0
        dx = self.model_l / 2
        dy = self.model_w / 2
        r = math.sqrt(dx ** 2 + dy ** 2)
        pts = [[position[0] + dx, position[1] + dy],
               [position[0] - dx, position[1] + dy],
               [position[0] - dx, position[1] - dy],
               [position[0] + dx, position[1] - dy]]
        for i in range(4):
            cv.line(self.image, self.dis2pixel(pts[i % 4]), self.dis2pixel(pts[(i + 1) % 4]), Color().Orange, 1)
        # TODO
        # for _ in range(4):
        #     pts.append([position[0] + r * cosd(origination), position[1] + r * cosd(origination)])
        # cv.fillConvexPoly(self.image, points=np.array([list(self.dis2pixel(pt)) for pt in pts]), color=Color().Orange)
        cv.imshow(self.name4image, self.image)
        cv.waitKey(1)

    def model_update(self, time_interval, u, display_iter = 10):
        # display = time_interval / display_iter
        for i in range(display_iter):
            self.state_update(u, time_interval / display_iter)
            self.draw_model(0, self.p)
            # cv.imshow(self.name4image, self.image)
        cv.circle(self.image, self.dis2pixel(self.p), 4, Color().Red, -1)
        cv.imshow(self.name4image, self.image)
        cv.waitKey(0)


if __name__ == '__main__':
    model_parameters = {
        'model_l': 0.3,
        'model_w': 0.3,
        'radius': None,
        'A': [
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ],
        'B': [
            [0, 0],
            [0, 0],
            [1, 0],
            [0, 1]
        ],
        's_dimension': 4,
        'p_dimension': 2,
        'u_dimension': 2,
        's_init': [
            5,
            5,
            0,
            0],
        'time_step': 0.01}

    map_parameters = {
        'width': 600,
        'height': 600,
        'x_size': 10,
        'y_size': 10,
        'image_name': 'kinodynamic',
        'start': [5, 5],
        'terminal': [10, 10],
        'obs': None,
        'map_file': None
    }

    kinodynamic = KinoDynamic(model_parameters, map_parameters)
    # print(kinodynamic.s)
    # print(kinodynamic.p)
    # print(kinodynamic.v)
    # print(kinodynamic.a)
    # print(kinodynamic.jerk)
    kinodynamic.draw_model(origination=0, position=kinodynamic.p)

    # tf = 4.0
    # N = int(tf/kinodynamic.time_step)
    # u = [[0, 0] for _ in range(N)]
    # kinodynamic.model_update(tf, u)
    # print('position:', kinodynamic.p)
    # print('velocity:', kinodynamic.v)
    # print('acceleration:', kinodynamic.a)
    # print('jerk:', kinodynamic.jerk)

    tf = 3.0
    control_max = 1
    control_step = 0.2
    u_x = np.arange(-control_max, control_max + control_step, control_step)
    u_y = np.arange(-control_max, control_max + control_step, control_step)
    for ux in u_x:
        for uy in u_y:
            N = int(tf / kinodynamic.time_step)
            u = [[ux, uy] for _ in range(N)]
            kinodynamic.model_update(tf, u)
            kinodynamic.reset(4, 2, [5, 5, 0, 0])
