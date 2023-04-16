import math

from Experiment.common.Point import Point


class AISPoint(Point):
    def __init__(self, x, y, trajectory_id=-1, t=-1, sog=-1, cog=-1):
        """
        初始化
        :param x:
        :param y:
        :param trajectory_id:
        :param t:
        :param sog:
        :param cog:
        """
        super().__init__(x, y, trajectory_id, t)
        self.sog = sog
        self.cog = cog
        self.v_lat = None
        self.v_lon = None

    def getV_by_sog_cog(self):
        v_lon = self.sog * math.sin(self.cog * math.pi / 180) * 1852 / 3600 / 100000  # 换算为米
        v_lat = self.sog * math.cos(self.cog * math.pi / 180) * 1852 / 3600 / 100000  # 换算为米
        return v_lat, v_lon

    def set_v(self, v_lat, v_lon):
        self.v_lon = v_lon
        self.v_lat = v_lat

    def get_v(self):
        return self.v_lat, self.v_lon
