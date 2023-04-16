# -*- coding: utf-8 -*-
import math
import numpy as np


class Point(object):
    def __init__(self, x, y, trajectory_id=-1, t=-1):
        """
        初始化 Point
        :param x: x坐标 纬度
        :param y: y坐标 经度
        :param trajectory_id: 轨迹 ID
        :param t: 时间戳 相对于第一个点的相对时间
        """
        self.trajectory_id = trajectory_id
        self.x = x
        self.y = y
        self.t = t
        self.p = None  # 指向参考点 即另一个 Point
        self.reference_trajectory = -1
        self.calculate_index = -1

    def __repr__(self):
        return "{0:.8f},{1:.8f}".format(self.x, self.y)

    def get_point(self):
        return self.x, self.y

    def __add__(self, other: 'Point'):
        if not isinstance(other, Point):
            raise TypeError("The other type is not 'Point' type.")
        _add_x = self.x + other.x
        _add_y = self.y + other.y
        return Point(_add_x, _add_y, trajectory_id=self.trajectory_id)

    def __sub__(self, other: 'Point'):
        if not isinstance(other, Point):
            raise TypeError("The other type is not 'Point' type.")
        _sub_x = self.x - other.x
        _sub_y = self.y - other.y
        return Point(_sub_x, _sub_y, trajectory_id=self.trajectory_id)

    def __mul__(self, x: float):
        if isinstance(x, float):
            return Point(self.x * x, self.y * x, trajectory_id=self.trajectory_id)
        else:
            raise TypeError("The other object must 'float' type.")

    def __truediv__(self, x: float):
        if isinstance(x, float):
            return Point(self.x / x, self.y / x, trajectory_id=self.trajectory_id)
        else:
            raise TypeError("The other object must 'float' type.")

    def dot(self, other: 'Point'):
        return self.x * other.x + self.y * other.y

    def to_list(self):
        return [int(self.t), self.x, self.y]

    def as_array(self):
        return np.array((self.x, self.y))

    def equal(self, other: 'Point') -> bool:
        """
        判断两个点的所有属性值是否完全相同
        :param other:
        :return:
        """
        return abs(self.x - other.x) < 1e-6 and abs(self.y - other.y) < 1e-6 and (self.t - other.t) < 1e-6

    def equal_position(self, other: 'Point') -> bool:
        """
        判断两个点的位置是否完全相同
        :param other:
        :return:
        """
        return abs(self.x - other.x) < 1e-6 and abs(self.y - other.y) < 1e-6

    def distance(self, other: 'Point') -> float:
        """
        计算两个point之间的距离
        :param other:
        :return: 返回两点的欧式距离值
        """
        return math.sqrt(math.pow(self.x - other.x, 2) + math.pow(self.y - other.y, 2))

    def point2line_distance(self, start: 'Point', end: 'Point') -> float:
        """
        计算 point 到 line 的垂直距离通过向量的方式: distance = |es x ps| / |es|, es为起始点的项量表示, ps为point到start点的向量
        :param start:线段起始点
        :param end:线段终点
        :return: float, point点到start, end两点连线的垂直距离, 欧式距离
        """
        point = self.as_array()
        start = start.as_array()
        end = end.as_array()
        if np.all(np.equal(start, end)):
            return np.linalg.norm(point - start)
        return np.divide(np.abs(np.linalg.norm(np.cross(end - start, start - point))),
                         np.linalg.norm(end - start))

    def point_intersect_line(self, start: 'Point', end: 'Point') -> 'Point':
        """
        求线段上与 self 点最近的点  交点在线段上 则返回交点 不在线段上 则返回线段两端距离点最近的点
        :param start: 线段起点
        :param end: 线段终点
        :return: 返回交点
        """
        x0, y0 = self.x, self.y
        x1, y1 = start.x, start.y
        x2, y2 = end.x, end.y
        if abs(y1 - y2) < 1e-5:
            if x0 < min(x1, x2) or x0 > max(x1, x2):
                dist_start_to_p = self.distance(start)
                dist_end_to_p = self.distance(end)
                if dist_start_to_p < dist_end_to_p:
                    return start
                else:
                    return end
            else:
                return Point(x=x0, y=y1)
        if abs(x1 - x2) < 1e-5:
            if y0 < min(y1, y2) or y0 > max(y1, y2):
                dist_start_to_p = self.distance(start)
                dist_end_to_p = self.distance(end)
                if dist_start_to_p < dist_end_to_p:
                    return start
                else:
                    return end
            else:
                return Point(x=x1, y=y0)
        k1 = (y2 - y1) / (x2 - x1)
        b1 = y1 - x1 * k1
        # 由 k0 = 1 / k1 改为 k0 = -1 / k1 6/18
        k0 = -1 / k1
        if k0 == k1:
            return start
        # ----
        b0 = y0 - x0 * k0
        intersect_x = round((b1 - b0) / (k0 - k1), 5)
        if intersect_x > max(x1, x2) or intersect_x < min(x1, x2):
            dist_start_to_p = self.distance(start)
            dist_end_to_p = self.distance(end)
            if dist_start_to_p < dist_end_to_p:
                return start
            else:
                return end
        return Point(intersect_x, intersect_x * k0 + b0)

    def linear_prediction(self, start: 'Point', end: 'Point') -> 'Point':
        """
        预测 self 在 线段start-end之间的位置
        :param start: 线段起点
        :param end: 线段终点
        :return: 预测 self 在线段 start end 之间的位置  返回预测点
        """
        lx = start.x + (self.t - start.t) / (end.t - start.t) * (end.x - start.x)
        ly = start.y + (self.t - start.t) / (end.t - start.t) * (end.y - start.y)
        return Point(lx, ly, t=self.t)

    def get_haversine(self, other: 'Point') -> float:
        """
        获得两个点之间的 真实距离（非欧氏距离）
        :param other: self 与 other 的距离
        :return: 距离值
        """
        EARTH_RADIUS = 6371229  # m 用于两点间距离计算
        lat1 = self.x * math.pi / 180
        lat2 = other.x * math.pi / 180
        lon1 = self.y * math.pi / 180
        lon2 = other.y * math.pi / 180
        d_lat = lat2 - lat1
        d_lon = lon2 - lon1
        a_a = math.sin(d_lat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a_a), math.sqrt(1 - a_a))
        return EARTH_RADIUS * c

    def line_intersect_circle(self, r0, lsp: 'Point', esp: 'Point') -> 'Point':
        """
        计算 圆（圆心self） 与 直接相交的点,如果不存在交点，返回左端点
        :param r0: 半径
        :param lsp: 线段左端点
        :param esp: 线段右端点
        :return: 交点 x,y 坐标
        """
        # p is the circle parameter, lsp and lep is the two end of the line
        x0, y0 = self.x, self.y
        x1, y1 = lsp.x, lsp.y
        x2, y2 = esp.x, esp.y
        if r0 == 0:
            return lsp
        if abs(self.distance(lsp) - r0) < 1e-5:
            return lsp
        if abs(self.distance(esp) - r0) < 1e-5:
            return esp
        if x1 == x2:
            if abs(r0) >= abs(x1 - x0):
                p1 = x1, round(y0 - math.sqrt(r0 ** 2 - (x1 - x0) ** 2), 5)
                p2 = x1, round(y0 + math.sqrt(r0 ** 2 - (x1 - x0) ** 2), 5)
                inp = [p1, p2]
                # select the points lie on the line segment
                inp = [p for p in inp if min(x1, x2) <= p[0] <= max(x1, x2)]
            else:
                inp = []
        else:
            k = (y1 - y2) / (x1 - x2)
            b0 = y1 - k * x1
            a = k ** 2 + 1
            b = 2 * k * (b0 - y0) - 2 * x0
            c = (b0 - y0) ** 2 + x0 ** 2 - r0 ** 2
            delta = b ** 2 - 4 * a * c
            if delta >= 0:
                p1x = round((-b - math.sqrt(delta)) / (2 * a), 5)
                p2x = round((-b + math.sqrt(delta)) / (2 * a), 5)
                p1y = round(k * p1x + b0, 5)
                p2y = round(k * p2x + b0, 5)
                inp = [[p1x, p1y], [p2x, p2y]]
                # select the points lie on the line segment
                inp = [p for p in inp if min(x1, x2) <= p[0] <= max(x1, x2)]
            else:
                inp = []

        return Point(inp[0][0], inp[0][1]) if inp != [] else lsp

    def get_point_by_time_and_line(self, time, end: 'Point') -> 'Point':
        """
        以 self 和 end 为线段起点和终点，预测 time 时刻的位置
        :param time: 需要预测的时间
        :param end: 线段终点
        :return: Point
        """
        x = self.x + (end.x - self.x) * (time - self.t) / (end.t - self.t)
        y = self.y + (end.y - self.y) * (time - self.t) / (end.t - self.t)
        return Point(x, y, t=time)

    def get_ped(self, left: 'Point', right: 'Point') -> float:
        """
        返回 self 到线段 left-right 的ped距离  函数功能与 point2line_distance 一致
        :param left: 线段左端点
        :param right: 线段右端点
        :return: ped距离
        """
        a = right.y - left.y
        b = left.x - right.x
        c = right.x * left.y - left.x * right.y
        # 如果left和right 相等 则返回 self 到 left 的距离，即两条直线的距离
        if a == 0 and b == 0:
            return self.distance(left)
        short_dist = abs((a * self.x + b * self.y + c) / math.sqrt(a * a + b * b))
        return short_dist

    def get_sed(self, left: 'Point', right: 'Point') -> float:
        """
        返回 self 到线段 left-right 的 sed 距离
        :param left:
        :param right:
        :return:
        """
        numerator = self.t - left.t
        denominator = right.t - left.t
        time_ratio = 1 if denominator == 0 else numerator / denominator
        new_x = left.x + (right.x - left.x) * time_ratio
        new_y = left.y + (right.y - left.y) * time_ratio
        # print([new_x, new_y])
        sed_error = math.sqrt((self.x - new_x) ** 2 + (self.y - new_y) ** 2)
        return sed_error

    def get_speed(self, other: 'Point') -> float:
        """
        获得该点与other点之间的平均速度
        :param other:
        :return:
        """
        return abs(self.distance(other) / (self.t - other.t))

    def get_angle(self, other: 'Point'):
        """
        self 与 other 相对于水平线的夹角
        :param other:
        :return:
        """
        # return math.atan2(other.y - self.y, other.x - self.x)
        return math.atan2(other.x - self.x, other.y - self.y)
