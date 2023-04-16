import time
import math
from queue import PriorityQueue
from typing import List
import numpy as np
import Experiment.Metrics.metrics as cr
import pandas as pd
from Experiment.common.Point import Point
from Experiment.common.zip import zip_compress
from Experiment.Metrics import metrics
from Experiment.data.data_process import get_trajectory_dataset


class Seg:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.max_score = 1
        self.target_index = -1
        self.col_max = None
        self.col_min = None
        self.target_row = None

    def __lt__(self, other: 'Seg'):
        return self.max_score < other.max_score


def get_seg(points: List[Point], start: int, end: int):
    if end == start or end == start + 1:
        return None
    res = Seg(start, end)
    score_matrix = []  
    for i in range(start + 1, end):
        # ped 
        ped_distance = points[i].get_ped(points[start], points[end])
        # sed 
        sed_distance = points[i].get_sed(points[start], points[end])
        # SAD
        predict_point_distance = abs(points[i].linear_prediction(points[start], points[end]).distance(
            points[i + 1].linear_prediction(points[start], points[end])))
        speed_distance = abs(predict_point_distance - points[i].distance(points[i + 1]))
        # DAD
        temp_direction = abs(points[start].get_angle(points[end]) - points[i].get_angle(points[i + 1]))
        direction_distance = abs(
            math.tan(min(temp_direction, 2 * math.pi - temp_direction)) * speed_distance)
        score_matrix.append([ped_distance, sed_distance, speed_distance, direction_distance])
    score_matrix = np.array(score_matrix)
    weight = np.array([1, 1, 1, 1])
    # Normalization
    score_list = np.arctan(score_matrix) * 2 / math.pi
    score_list = score_list.dot(weight)
    index = np.where(score_list == np.max(score_list))[0][0]
    res.target_index = start + 1 + index
    res.max_score = -1 * score_list.max()
    return res


def priority_dp(points: List[Point], compression_rate: float):
    """

    :param points:
    :param compression_rate:
    :return:
    """
    remaining_points = math.ceil(len(points) * (1 - compression_rate)) - 2  # 起点和终点必要
    queue_seg = PriorityQueue()  # 存放各段信息
    queue_sample = PriorityQueue()  # 存放最后返回的简化轨迹的点的索引
    # 放入起点和终点
    queue_sample.put(0)
    queue_sample.put(len(points) - 1)
    while remaining_points > 0:
        if queue_seg.qsize() == 0:
            # 初始获取起点到终点的 seg 信息
            cur_seg = get_seg(points, 0, len(points) - 1)
        else:
            cur_seg = queue_seg.get()
        # 获得切割点
        target_index = cur_seg.target_index
        queue_sample.put(target_index)
        # 更新优先级
        left_seg = get_seg(points, cur_seg.start, target_index)
        right_seg = get_seg(points, target_index, cur_seg.end)
        if left_seg is not None:
            queue_seg.put(left_seg)
        if right_seg is not None:
            queue_seg.put(right_seg)
        remaining_points -= 1
    ret = []
    while queue_sample.qsize() > 0:
        ret.append(queue_sample.get())
    return ret


def douglas_peucker(points: List[Point], start: int, last: int, epsilon: float):
    """
        DP 算法
    :param points:
    :param start:
    :param last:
    :param epsilon: 误差阈值  epsilon为欧氏距离   epsilon*100000≈实际距离
    :return: 简化后的索引序列
    """
    d_max = 0
    index = start
    rec_result = []
    for i in range(start + 1, last):
        d = points[i].get_ped(points[start], points[last])
        if d > d_max:
            index = i
            d_max = d
    if d_max > epsilon:
        rec_result1 = douglas_peucker(points, start, index, epsilon)
        rec_result2 = douglas_peucker(points, index, last, epsilon)
        rec_result.extend(rec_result1)
        rec_result.extend(rec_result2[1:])
    else:
        rec_result.append(start)
        rec_result.append(last)
    return rec_result


# TD-TR
def td_tr(points: List[Point], start: int, last: int, epsilon: float) -> list:
    """
    td-tr 算法
    :param points:
    :param start:
    :param last:
    :param epsilon:
    :return:
    """
    d_max = 0
    index = start
    rec_result = []
    for i in range(start + 1, last):
        d = points[i].get_sed(points[start], points[last])
        if d > d_max:
            index = i
            d_max = d
    if d_max > epsilon:
        rec_result1 = td_tr(points, start, index, epsilon)
        rec_result2 = td_tr(points, index, last, epsilon)
        rec_result.extend(rec_result1)
        # the first point must can't join because it equals with the last point of rec_result1
        rec_result.extend(rec_result2[1:])
    else:
        rec_result.append(start)
        rec_result.append(last)
    return rec_result


def run():
    # res = []
    # dataset_name = "BerlinMOD_0_005_data"
    # dataset_name = "walk_data"
    dataset_name = "airline_data"
    # dataset_name = "illinois_data"
    point_remain = 0
    point_all = 0
    trajectories = get_trajectory_dataset(dataset_name)
    for trajectory in trajectories:
        point_all += len(trajectory.points)
        dp_trajectory = []
        point_remain += len(sample_index)
        for e in sample_index:
            dp_trajectory.append(trajectory.points[e])
        trajectory.points = dp_trajectory
        dp_trajectories.append(dp_trajectory)

if __name__ == '__main__':
    run()
