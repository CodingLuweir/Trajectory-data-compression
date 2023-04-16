import numpy as np
import pandas as pd
from typing import List

from Experiment.common import AISPoint
from Experiment.common.Point import Point


# ---------------------------------------------
# author: luweir
# target: trajectory class
# date: 2022-5-13
# ---------------------------------------------

class Trajectory:
    trajectory_id = -1  # 轨迹自身ID
    points: List[Point] = []  # 自身轨迹点
    reference_trajectory_id = -1  # 参考轨迹ID
    reference_time = []  # 对应在参考轨迹上的时间

    def __init__(self, trajectory_id, points=None):
        self.trajectory_id = trajectory_id
        self.reference_trajectory_id = -1
        self.reference_time = []
        self.points = [] if points is None else points

    def add_point(self, point):
        self.points.append(point)

    def to_list(self):
        """
        将 self.points 转换成 [[]] 的形式输出
        :return:
        """
        trajectory_list = []
        for ele in self.points:
            trajectory_list.append([ele.t, ele.x, ele.y])
        return trajectory_list


