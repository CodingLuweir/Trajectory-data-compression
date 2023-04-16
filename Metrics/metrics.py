from typing import List
import math
from fastdtw import fastdtw
from Experiment.common.Point import Point


def get_PED_error(point: List[Point], sample: List[Point]) -> list:
    """
    返回压缩轨迹和原始轨迹的 PED 误差
    :param point: 原始轨迹
    :param sample: 简化轨迹
    :return: [平均误差，最大误差]
    """
    left_point = sample[0]
    sample_index = 1
    point_index = 0
    max_ped_error = 0
    sum_ped_error = 0
    while sample_index < len(sample):
        right_point = sample[sample_index]
        # 如果当前点在简化后的两点之间
        while point_index < len(point) and left_point.t <= point[point_index].t < right_point.t:
            cur_point = point[point_index]
            # 计算PED误差
            ped_error = 0 if cur_point.equal(left_point) else cur_point.get_ped(left_point, right_point)
            sum_ped_error += ped_error
            max_ped_error = max(max_ped_error, ped_error)
            point_index += 1
        # 简化轨迹的起点后移
        sample_index += 1
        left_point = right_point

    return [sum_ped_error / len(point), max_ped_error]


def get_SED_error(point: List[Point], sample: List[Point]) -> list:
    """
    返回压缩轨迹 和 原始轨迹的 SED 误差
    :param point: 原始轨迹
    :param sample: 简化轨迹
    :return: [平均SED误差，最大SED误差]
    """
    left_point = sample[0]
    sample_index = 1
    point_index = 0
    max_sed_error = 0
    sum_sed_error = 0
    while sample_index < len(sample):
        right_point = sample[sample_index]
        # 如果当前点在简化后的两点之间
        while point_index < len(point) and left_point.t <= point[point_index].t < right_point.t:
            cur_point = point[point_index]
            # 计算SED误差 如果左右点位置相同 那么就计算cur_point与二者任一点的距离
            sed_error = 0 if cur_point.equal(left_point) else cur_point.get_sed(left_point, right_point)
            sum_sed_error += sed_error
            max_sed_error = max(max_sed_error, sed_error)
            point_index += 1
        # 简化轨迹起点后移
        sample_index += 1
        left_point = right_point

    return [sum_sed_error / len(point), max_sed_error]


def get_speed_error(point: List[Point], sample: List[Point]) -> list:
    """
    获得原始轨迹与压缩轨迹的速度误差
    :param point:
    :param sample:
    :return:[平均速度误差，最大速度误差]
    """
    left_point = sample[0]
    sample_index = 1
    point_index = 0
    max_speed_error = 0
    sum_speed_error = 0
    while sample_index < len(sample):
        right_point = sample[sample_index]
        # 如果当前点在简化后的两点之间
        while point_index < len(point) and left_point.t <= point[point_index].t < right_point.t:
            # 计算speed误差
            speed_error = 0
            if left_point.t != point[point_index].t:
                speed_error = abs(
                    point[point_index].get_speed(point[point_index + 1]) - left_point.get_speed(right_point))
            else:
                speed_error = abs(
                    point[point_index].get_speed(point[point_index + 1]) - left_point.get_speed(right_point))
            max_speed_error = max(max_speed_error, speed_error)
            sum_speed_error += speed_error
            point_index += 1
        sample_index += 1
        left_point = right_point

    # =============================================
    # while sample_index < len(sample):
    #     right_point = sample[sample_index]
    #     # 如果当前点在简化后的两点之间
    #     while point_index < len(point) - 1 and left_point.t <= point[point_index].t < right_point.t:
    #         simulate_point1 = point[point_index].linear_prediction(left_point, right_point)
    #         simulate_point2 = point[point_index + 1].linear_prediction(left_point, right_point)
    #         if simulate_point1.t != simulate_point2.t:
    #             # 计算speed误差
    #             speed_error = abs(
    #                 point[point_index].get_speed(point[point_index + 1]) - simulate_point1.get_speed(simulate_point2))
    #             max_speed_error = max(max_speed_error, speed_error)
    #             sum_speed_error += speed_error
    #         point_index += 1
    #         if point_index >= len(point):
    #             return [sum_speed_error / len(point), max_speed_error]
    #     sample_index += 1
    #     left_point = right_point

    return [sum_speed_error / len(point), max_speed_error]


def get_angle_error(point: List[Point], sample: List[Point]) -> list:
    """
    获得原始轨迹 与 压缩轨迹的角度误差
    :param point:
    :param sample:
    :return:[平均角度误差，最大角度误差]
    """
    left_point = sample[0]
    sample_index = 1

    point_index = 0
    max_angle_error = 0
    sum_angle_error = 0
    while sample_index < len(sample):
        right_point = sample[sample_index]
        while point_index < len(point) and left_point.t <= point[point_index].t < right_point.t:
            cur_point = point[point_index]
            # 计算angle误差
            temp_direction = abs(
                left_point.get_angle(right_point) - point[point_index].get_angle(point[point_index + 1]))
            angle_error = 0
            if left_point.t != point[point_index].t:
                angle_error = min(temp_direction, 2 * math.pi - temp_direction)
            else:
                angle_error = min(temp_direction, 2 * math.pi - temp_direction)
            max_angle_error = max(max_angle_error, angle_error)
            sum_angle_error += angle_error
            point_index += 1
        sample_index += 1
        left_point = right_point
    # ==========================================================
    # while sample_index < len(sample):
    #     right_point = sample[sample_index]
    #     while point_index < len(point) and left_point.t <= point[point_index].t < right_point.t:
    #         cur_point = point[point_index]
    #         # 计算angle误差
    #         angle_error = 0 if left_point.t == point[point_index].t or left_point.equal_position(cur_point) else abs(
    #             cur_point.get_angle(point[point_index + 1]) - left_point.get_angle(right_point))
    #         max_angle_error = max(max_angle_error, angle_error)
    #         sum_angle_error += angle_error
    #         point_index += 1
    #     sample_index += 1
    #     left_point = right_point
    # ===========================================================
    # while sample_index < len(sample):
    #     right_point = sample[sample_index]
    #     sample_angle = math.atan2(right_point.y - left_point.y, right_point.x - left_point.x)
    #     while left_point.t <= point[point_index].t < right_point.t:
    #         cur_point = point[point_index]
    #         if cur_point.t != left_point.t:
    #             # 计算angle误差
    #             angle_error = abs(
    #                 abs(sample_angle) - abs(math.atan2(cur_point.y - left_point.y, cur_point.x - left_point.x)))
    #             max_angle_error = max(max_angle_error, angle_error)
    #             sum_angle_error += angle_error
    #         point_index += 1
    #         if point_index >= len(point):
    #             return [sum_angle_error / len(point), max_angle_error]
    #     sample_index += 1
    #     left_point = right_point
    return [sum_angle_error / len(point), max_angle_error]


def get_haversine(point_a, point_b):
    EARTH_RADIUS = 6371229  # m 用于两点间距离计算
    lat1 = point_a[1] * math.pi / 180
    lat2 = point_b[1] * math.pi / 180
    lon1 = point_a[2] * math.pi / 180
    lon2 = point_b[2] * math.pi / 180
    d_lat = lat2 - lat1
    d_lon = lon2 - lon1
    a_a = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a_a), math.sqrt(1 - a_a))
    return EARTH_RADIUS * c


def get_dtw(point: List[Point], sample: List[Point]):
    t1 = []
    t2 = []
    for ele in point:
        t1.append(ele.to_list())
    for ele in sample:
        t2.append(ele.to_list())
    distance, path = fastdtw(t1, t2, dist=get_haversine)
    return distance / 1000.0


if __name__ == '__main__':
    a = [Point(x=1.1, y=2.1, t=0), Point(x=1.5, y=2.1, t=2)]
    b = [Point(x=1.1, y=2.1, t=0), Point(x=1.3, y=2.1, t=1), Point(x=1.5, y=2.1, t=2)]
    print(get_dtw(a, b))
