class Color:
    def __init__(self):
        """
        :brief:     初始化
        """
        self.Black = (0, 0, 0)
        self.White = (255, 255, 255)

        self.Blue = (255, 0, 0)
        self.Green = (0, 255, 0)
        self.Red = (0, 0, 255)

        self.Yellow = (0, 255, 255)
        self.Cyan = (255, 255, 0)
        self.Magenta = (255, 0, 255)

        self.DarkSlateBlue = (139, 61, 72)
        self.LightPink = (193, 182, 255)
        self.Orange = (0, 165, 255)
        self.DarkMagenta = (139, 0, 139)
        self.Chocolate2 = (33, 118, 238)
        self.Thistle = (216, 191, 216)
        self.Purple = (240, 32, 160)
        self.DarkGray = (169,169,169)
        self.Gray = (128, 128, 128)
        self.DimGray = (105,105,105)
        self.DarkGreen = (0, 100, 0)

        self.n_color = 19

        self.color_container = [self.Black,
                                self.White,
                                self.Blue,
                                self.Green,
                                self.Red,
                                self.Yellow,
                                self.Cyan,
                                self.Magenta,
                                self.DarkSlateBlue,
                                self.LightPink,
                                self.Orange,
                                self.DarkMagenta,
                                self.Chocolate2,
                                self.Thistle,
                                self.Purple,
                                self.DarkGray,
                                self.Gray,
                                self.DimGray,
                                self.DarkGreen]

    def get_color_by_item(self, _n: int):
        """
        :brief:         通过序号索引颜色
        :param _n:      序号
        :return:        返回的颜色
        """
        assert 0 <= _n < self.n_color   # 当 _n 不满足时触发断言
        return self.color_container[_n]
