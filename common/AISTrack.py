from typing import List

from Experiment.common.AISPoint import AISPoint
from Experiment.common.Trajectory import Trajectory


class AISTrack(Trajectory):
    ais_points: List[AISPoint] = []  # 自身轨迹点

    def __init__(self, trajectory_id, ais_points=None, MMSI=-1):
        super().__init__(trajectory_id)
        self.ais_points = ais_points
        self.MMSI = MMSI

    def add_point(self, point):
        self.ais_points.append(point)

    def to_list(self):
        """
        将 self.points 转换成 [[]] 的形式输出
        :return:
        """
        trajectory_list = []
        for ele in self.ais_points:
            trajectory_list.append([ele.t, ele.x, ele.y, ele.sog, ele.cog])
        return trajectory_list
