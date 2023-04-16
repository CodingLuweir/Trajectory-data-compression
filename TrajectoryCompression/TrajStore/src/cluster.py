import numpy as np
import pandas as pd
import math
import sys

from typing import List

from Experiment.common.Trajectory import Trajectory
from Experiment.common.Point import Point


def traj_store_cluster(trajectories: List[Trajectory], epsilon=1.0) -> list:
    """
    对若干个轨迹进行聚类  原理：共用空间信息
    :param trajectories: 轨迹集
    :param epsilon: 误差阈值 epsilon越大，越可能聚到一起
    :return: 聚类结果 list
    """
    # 一、聚类
    group = []
    trajectory_set = set()
    for trajectory in trajectories:
        # 当前轨迹已经归类 则跳过
        if trajectory.trajectory_id in trajectory_set:
            continue
        # 否则 进行聚类 以自身为中心轨迹 找其它跟它相似的轨迹
        cluster: List[Trajectory] = [trajectory]
        trajectory_set.add(trajectory.trajectory_id)
        for other in trajectories:
            if other.trajectory_id in trajectory_set:
                continue
            # 如果二者距离满足阈值 则聚成一类
            if dist(trajectory, other) <= epsilon:
                cluster.append(other)
                trajectory_set.add(other.trajectory_id)
        group.append(cluster)
    # 二、对聚类后的轨迹簇 进行多轨迹的压缩  =>  即将非参考轨迹的 (t,x,y) 转变为 t'
    # 注意 映射时间是在非参考轨迹那里 不是在参考轨迹那里
    for cluster in group:
        # 如果该聚簇没有参考轨迹  原始存储
        if len(cluster) == 1:
            continue
        # 该聚簇有参考轨迹  选最长的作为参考轨迹  以获得最佳时间映射
        reference_trajectory = cluster[0]
        # 对非参考轨迹进行压缩
        for i in range(len(cluster)):
            trajectory = cluster[i]
            if trajectory == reference_trajectory:
                continue
            trajectory.reference_trajectory_id = reference_trajectory.trajectory_id
            traj_dist(reference_trajectory, trajectory, True)
    return group


def traj_dist(t1: Trajectory, t2: Trajectory, pattern=False) -> float:
    """
    计算轨迹 t1 和 t2 的最大距离   距离 = max (t1各点 与 对应的 t2中的点 的欧式距离)
    :param t1: 轨迹1
    :param t2: 轨迹2
    :param pattern: False 时仅计算距离；True 时进行时间映射  t1的位置映射为t2的时间
    :return:
    """
    max_dist = -1
    # 一、 对于t1中的每个点 Pi 计算 t2中对应的位置 Pi'
    for i in range(len(t1.points)):
        # print("i: ", i)
        t1_point = t1.points[i]
        # 1.1 计算 Pi 到第一个点的线性距离 distance_for_pi
        distance_for_pi = t1_point.distance(t1.points[0])

        # 1.2 找到 t2 中距离第一个点 distance_for_pi 距离的位置  本质是一个圆与一个线段的交点
        # 非映射点：隔 t1_point 最近的两个点 也需要存储  防止在t2中找不到对应点
        nc_point_closed_index = 0
        nc_point_closed_dist = sys.maxsize

        # 找对应点开始
        index = 0
        index_set = set()
        while index < len(t2.points) - 1:
            pre_dist = t2.points[index].distance(t1_point)
            aft_dist = t2.points[index + 1].distance(t1_point)
            if (pre_dist + aft_dist) / 2 < nc_point_closed_dist:
                nc_point_closed_index = index
                nc_point_closed_dist = (pre_dist + aft_dist) / 2
            pre_dist = t2.points[index].distance(t2.points[0])
            aft_dist = t2.points[index + 1].distance(t2.points[0])
            # 线性距离在 两点之间  为了保证精度 每个对应点位置的确定都遍历一遍 t2
            if (pre_dist <= distance_for_pi <= aft_dist) or (pre_dist >= distance_for_pi >= aft_dist):
                index_set.add(index)
            index += 1

        # 二、计算 Pi 与 Pi' 的欧氏距离
        # 如果 distance_for_pi 已经超出了 t2 的范围   即找不到对应点  选择nc_point_closed_index
        if index == len(t2.points) - 1 and len(index_set) == 0:
            line_pre = t2.points[nc_point_closed_index]
            line_next = t2.points[nc_point_closed_index + 1]
            nc_point = t1_point.point_intersect_line(line_pre, line_next)
            dist1 = t1_point.get_haversine(nc_point)
            dist2 = t1_point.get_haversine(t2.points[index])
            # 如果非最佳映射点 隔 t1_point 距离 小于 最后一个点 隔 t1_point 的距离 那就存前者
            if dist1 < dist2:
                max_dist = max(max_dist, dist1)
                delta_t = 0
                if line_pre.get_haversine(line_next) > 1e-6:
                    delta_t = line_pre.get_haversine(nc_point) / line_pre.get_haversine(line_next) * (
                            line_next.t - line_pre.t)
                if pattern:
                    t2.reference_time.append(line_pre.t + delta_t)
            # 否则就存后者
            else:
                max_dist = max(max_dist, dist2)
                if pattern:
                    t2.reference_time.append(t2.points[index].t)
        else:
            # 范围内 找对应点
            circle_point = t2.points[0]
            best_closed_distance = sys.maxsize
            best_position = 0
            corresponding_point = None
            # 找出其中最接近原始位置的 映射点
            for position in index_set:
                line_pre = t2.points[position]
                line_next = t2.points[position + 1]
                corresponding_point = circle_point.line_intersect_circle(distance_for_pi, line_pre, line_next)
                if t1_point.get_haversine(corresponding_point) < best_closed_distance:
                    best_position = position
                    best_closed_distance = t1_point.get_haversine(corresponding_point)

            # 计算与原始位置的距离
            line_pre = t2.points[best_position]
            line_next = t2.points[best_position + 1]

            # 如果 非映射点 比 映射点 的距离还近  必然选择非映射点
            nc_point = t1_point.point_intersect_line(t2.points[nc_point_closed_index],
                                                     t2.points[nc_point_closed_index + 1])
            if nc_point.get_haversine(t1_point) < corresponding_point.get_haversine(t1_point):
                line_pre = t2.points[nc_point_closed_index]
                line_next = t2.points[nc_point_closed_index + 1]
                corresponding_point = nc_point
            # 到这里 line_pre 和 line_next 指向了隔 t1_point 最近的点的端点 corresponding_point为最近的点
            if pattern:
                delta_t = line_pre.get_haversine(corresponding_point) / line_pre.get_haversine(line_next) * (
                        line_next.t - line_pre.t)
                t2.reference_time.append(line_pre.t + delta_t)
            max_dist = max(t1_point.get_haversine(corresponding_point), max_dist)
    # 3）所有点中最大的距离 即为 return 值
    # print(max_dist)
    return max_dist


# 计算轨迹间的距离 t1->t2 t2-t1 取距离最大值
def dist(t1, t2):
    return max(traj_dist(t2, t1), traj_dist(t1, t2))
