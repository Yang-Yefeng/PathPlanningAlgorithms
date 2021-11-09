import math


def sind(theta):
    return math.sin(theta / 180.0 * math.pi)

def cosd(theta):
    return math.cos(theta / 180.0 * math.pi)


class obstacle:
    def __init__(self, obs):
        self.name_set = ['triangle',        # 三角形(等腰)(就是因为描述方便)(格式统一)
                         'rectangle',       # 四边形(利用矩形外接圆的方式去定义)
                         'pentagon',        # 五边形(正)(利用矩形外接圆的方式去定义)
                         'hexagon',         # 六边形(正)(利用矩形外接圆的方式去定义)
                         'circle',          # 圆形
                         'ellipse']         # 椭圆形
        self.obs = self.set_obs(obs)        # the formation is ['name', [r], [points]]
        ''' triangle        ['triangle',  [pt1, pt2], [r, theta0, theta_bias]]     pt should be clockwise or counter-clock wise 
            rectangle       ['rectangle', [pt1, pt2], [r, theta0, theta_bias]]             pt1 and pt2 are the coordinate of the center
            pentagon        ['pentagon',  [pt1, pt2], [r, theta_bias]]
            hexagon         ['hexagon',   [pt1, pt2], [r, theta_bias]]
            circle          ['circle',    [pt1, pt2], [r]]
            ellipse         ['ellipse',   [pt1, pt2], [long_axis, short_axis, theta_bias]]'''

    def set_obs(self, message: list):
        obs = []
        if len(message) == 0:
            pass
        for item in message:
            if item == []:
                continue
            [name, [x, y], constraints] = item
            if name == 'triangle':              # ['triangle',  [pt1, pt2], [r, theta0, theta_bias]]
                [r, theta0, theta_bias] = constraints
                pt1 = [x + r * cosd(90 + theta_bias), y + r * sind(90 + theta_bias)]
                pt2 = [x + r * cosd(270 - theta0 + theta_bias), y + r * sind(270 - theta0 + theta_bias)]
                pt3 = [x + r * cosd(theta0 - 90 + theta_bias), y + r * sind(theta0 - 90 + theta_bias)]
                obs.append([name, [x, y, r], [pt1, pt2, pt3]])
            elif name == 'rectangle':
                [r, theta0, theta_bias] = constraints
                pt1 = [x + r * cosd(theta0 + theta_bias), y + r * sind(theta0 + theta_bias)]
                pt2 = [x + r * cosd(180 - theta0 + theta_bias), y + r * sind(180 - theta0 + theta_bias)]
                pt3 = [x + r * cosd(180 + theta0 + theta_bias), y + r * sind(180 + theta0 + theta_bias)]
                pt4 = [x + r * cosd(-theta0 + theta_bias), y + r * sind(-theta0 + theta_bias)]
                obs.append([name, [x, y, r], [pt1, pt2, pt3, pt4]])
            elif name == 'pentagon':
                [r, theta_bias] = constraints
                pt = []
                for i in range(5):
                    pt.append([x + r * cosd(90 + 72 * i + theta_bias), y + r * sind(90 + 72 * i + theta_bias)])
                obs.append([name, [x, y, r], pt])
            elif name == 'hexagon':
                [r, theta_bias] = constraints
                pt = []
                for i in range(6):
                    pt.append([x + r * cosd(90 + 60 * i + theta_bias), y + r * sind(90 + 60 * i + theta_bias)])
                obs.append([name, [x, y, r], pt])
            elif name == 'circle':
                obs.append([name, constraints, [x, y]])
            elif name == 'ellipse':
                obs.append([name, constraints, [x, y]])
            else:
                print('Unknown obstacle type')
        return obs

    def get_obs(self):
        return self.obs