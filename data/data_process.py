import datetime
import os

from dateutil import parser
import random
import pandas as pd
from typing import List

from Experiment.common.AISPoint import AISPoint
from Experiment.common.AISTrack import AISTrack
from Experiment.common.Point import Point
from Experiment.common.Trajectory import Trajectory


def load_trajectory(data: pd.DataFrame, trajectoryId: int = -1, format=1):
    """
    加载 单条轨迹 1=> t x y;2=> x y t;3=>AIS x y t;4=>AIS x y t seg cog
    :param format:
    :param data:
    :param trajectoryId:
    :return:
    """
    points = []
    if format == 1:
        # 格式 t x(lat) y(lon)
        start_time = parser.parse(data.iloc[0][0])

        for i in range(len(data)):
            cur_point = Point(x=data.iloc[i][1], y=data.iloc[i][2],
                              t=(parser.parse(data.iloc[i][0]) - start_time).total_seconds(),
                              trajectory_id=trajectoryId)
            points.append(cur_point)
    elif format == 2:
        # 格式  x(lat) y(lon) t
        start_time = parser.parse(data.iloc[0][2])
        for i in range(len(data)):
            cur_point = Point(x=data.iloc[i][0], y=data.iloc[i][1],
                              t=(parser.parse(data.iloc[i][2]) - start_time).total_seconds(),
                              trajectory_id=trajectoryId)
            points.append(cur_point)
    elif format == 3:
        # AIS数据集 格式 mmsi  y(lon) x(lat)  t sog 航首 cog
        start_time = parser.parse(data.iloc[0][3])
        for i in range(len(data)):
            cur_point = Point(x=data.iloc[i][2], y=data.iloc[i][1],
                              t=(parser.parse(data.iloc[i][3]) - start_time).total_seconds(),
                              trajectory_id=trajectoryId)
            points.append(cur_point)
    elif format == 4:
        # AIS数据集 格式 mmsi  y(lon) x(lat)  t sog 航首 cog
        start_time = parser.parse(data.iloc[0][3])
        for i in range(len(data)):
            # seconds获得的秒只是时间差中的小时、分钟和秒部分的和，并没有包含时间差的天数（既是两个时间点不是同一天，失效）
            # 所以要用 total_seconds()
            cur_point = AISPoint(x=data.iloc[i][2], y=data.iloc[i][1],
                                 t=(parser.parse(data.iloc[i][3]) - start_time).total_seconds(),
                                 sog=data.iloc[i][4],
                                 cog=data.iloc[i][6], trajectory_id=trajectoryId)
            points.append(cur_point)
    elif format == 5:
        # AIS数据集 格式 ID  x(lat) y(lon) sog cog t
        start_time = parser.parse(data.iloc[0][5])
        for i in range(len(data)):
            # seconds获得的秒只是时间差中的小时、分钟和秒部分的和，并没有包含时间差的天数（既是两个时间点不是同一天，失效）
            # 所以要用 total_seconds()
            cur_point = AISPoint(x=data.iloc[i][1], y=data.iloc[i][2],
                                 t=(parser.parse(data.iloc[i][5]) - start_time).total_seconds(),
                                 sog=data.iloc[i][3],
                                 cog=data.iloc[i][4], trajectory_id=trajectoryId)
            points.append(cur_point)
    return points


def get_trajectory_dataset(dataset_name: str, num: int = -1, MMSI=-1, path=None):
    """
    根据 dataset_name 获取数据级数据，都是以 Trajectory 列表的形式返回 num为数据集内加载的轨迹数量 -1表示所有轨迹
    :param path:
    :param MMSI:
    :param num: 数据集内加载的轨迹数量 -1表示所有轨迹
    :param dataset_name: 数据集名称
    :return:
    """
    trajectories = []

    if dataset_name == "BerlinMOD_0_005_data":
        path = r'E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\data\BerlinMOD_0_005Data\select_trajectory_handle_'
        TRAJECTORY_LEN = 10
        for i in range(TRAJECTORY_LEN):
            csv_dataset = pd.read_csv(path + str(i + 1) + '.txt', header=None)
            trajectories.append(Trajectory(trajectory_id=i, points=load_trajectory(data=csv_dataset, trajectoryId=i)))
            print("加载第", i, "条完成")
            if num != -1 and i >= num - 1:
                break

    elif dataset_name == "walk_data":
        path = r'E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\data\WalkingData'
        filenames = ["5-31-1", "5-31-2", "6-1-1", "6-1-2", "6-2-1", "6-2-2", "6-3-1", "6-3-2", "6-4-2"]
        for i in range(len(filenames)):
            trajectories.append(Trajectory(trajectory_id=i, points=load_trajectory(
                pd.read_csv(path + "\\" + filenames[i] + '.txt', header=None, sep='\t'), trajectoryId=i)))
            print("加载第", i, "条完成")
            if num != -1 and i >= num - 1:
                break

    elif dataset_name == "airline_data":
        path = r'E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\data\AirlineData'
        index = 0
        for i in range(9, 26):
            trajectories.append(Trajectory(trajectory_id=index, points=load_trajectory(
                pd.read_csv(path + "\\10." + str(i) + '.txt', header=None, sep='\t'), trajectoryId=index)))
            index += 1
            print("加载第", index, "条完成")
            if num != -1 and index >= num - 1:
                break
    elif dataset_name == "illinois_data":
        path = r'E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\data\IllinoisData\processd_data'
        trajectories = []
        data_files = os.listdir(path)
        index = 0
        for data_file in data_files:
            trajectories.append(Trajectory(trajectory_id=index, points=load_trajectory(
                pd.read_csv(path + "\\" + data_file, header=None, sep=','), trajectoryId=index, format=2)))
            print("加载第", index, "条轨迹：" + str(data_file) + "完成")
            index += 1
            if num != -1 and index > num - 1:
                break
    elif dataset_name == "AISTrajectory":
        path = r'D:\DeskTop\PythonFile\PythonProject\Experiment\data\AISData\chinaports'
        data_files = os.listdir(path)
        index = 0
        trajectories = []
        for data_file in data_files:
            points = load_trajectory(pd.read_csv(path + "\\" + data_file, sep=','), trajectoryId=index, format=3)
            trajectories.append(Trajectory(trajectory_id=index, points=points))
            print("加载第", index, "条完成")
            index += 1
            if num != -1 and index >= num - 1:
                break
    elif dataset_name == "AISTrack":
        AISTracks = []
        if path is None:
            path = r'D:\DeskTop\PythonFile\PythonProject\Experiment\data\AISData\processed'
        if MMSI != -1:
            path = path + "\\" + str(MMSI) + '.csv'
            points = load_trajectory(pd.read_csv(path, sep=','), trajectoryId=-1, format=4)
            AISTracks.append(AISTrack(trajectory_id=-1, ais_points=points))
        else:
            data_files = os.listdir(path)
            index = 0
            for data_file in data_files:
                data = pd.read_csv(path + "\\" + data_file, sep=',')
                points = load_trajectory(data, trajectoryId=data.iloc[0, 0], format=4)
                AISTracks.append(AISTrack(trajectory_id=data.iloc[0, 0], ais_points=points))
                print("加载第", index, "条完成")
                index += 1
                if num != -1 and index >= num:
                    break
        trajectories = AISTracks
    elif dataset_name == "AISTrack2":
        AISTracks = []
        if path is None:
            path = r'E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\data\Offshore'
        if MMSI != -1:
            path = path + "\\" + str(MMSI) + '.csv'
            points = load_trajectory(pd.read_csv(path, sep=','), trajectoryId=-1, format=5)
            AISTracks.append(AISTrack(trajectory_id=-1, ais_points=points))
        else:
            data_files = os.listdir(path)
            index = 0
            for data_file in data_files:
                data = pd.read_csv(path + "\\" + data_file, sep=',')
                points = load_trajectory(data, trajectoryId=data.iloc[0, 0], format=5)
                AISTracks.append(AISTrack(trajectory_id=data.iloc[0, 0], ais_points=points))
                print("加载第", index, "条完成")
                index += 1
                if num != -1 and index >= num:
                    break
        trajectories = AISTracks

    return trajectories


def handle_berlin_mod_0_005_dataset():
    """
    单独运行处理 柏林 0.005 数据
    1、BerlinMOD_0_005Data数据集中 相同时间不同/相同距离的点的问题
    2、BerlinMOD_0_005Data数据集中 出现时间跳跃的问题
    :return:
    """
    path = r'E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\data\BerlinMOD_0_005Data\select_trajectory_'
    TRAJECTORY_LEN = 10
    for i in range(TRAJECTORY_LEN):
        csv_dataset = pd.read_csv(path + str(i + 1) + '.txt', header=None)
        # csv_dataset = pd.read_csv(path + 'test.txt', header=None)
        accum_time = 0
        start_time = parser.parse(csv_dataset.iloc[0][0])
        trajectory_start_time = datetime.strptime(str(start_time), "%Y-%m-%d %H:%M:%S")
        trajectory = []
        pre_point = Point(x=0, y=0, t=-1)
        for j in range(len(csv_dataset)):
            cur_point = Point(x=csv_dataset.iloc[j][1], y=csv_dataset.iloc[j][2],
                              t=(parser.parse(csv_dataset.iloc[j][0]) - start_time).seconds + accum_time,
                              trajectory_id=i)
            # 防止 BerlinMOD_0_005Data 数据集中 相同时间不同位置的点的问题
            if cur_point.t == pre_point.t:
                cur_point.t = pre_point.t + 1
                if cur_point.equal_position(pre_point):
                    cur_point.x = round((pre_point.x + csv_dataset.iloc[j + 1][1]) / 2, 5)
                    cur_point.y = round((pre_point.y + csv_dataset.iloc[j + 1][2]) / 2, 5)
            # 防止BerlinMOD_0_005Data 数据集中 出现时间跳跃的问题 时间还是按之前累计，跳跃点按上下两点取均值
            elif cur_point.t > pre_point.t + 100 and cur_point.equal_position(pre_point):
                start_time = parser.parse(csv_dataset.iloc[j][0])
                accum_time = pre_point.t
                cur_point.t = accum_time + 1
                cur_point.x = round((pre_point.x + csv_dataset.iloc[j + 1][1]) / 2, 5)
                cur_point.y = round((pre_point.y + csv_dataset.iloc[j + 1][2]) / 2, 5)

            trajectory.append(cur_point)
            pre_point = cur_point

        # 输出
        for j in range(len(trajectory)):
            # if j == 72 or j == 665:
            #     print(11)
            csv_dataset.iat[j, 0] = (trajectory_start_time + datetime.timedelta(seconds=trajectory[j].t)).strftime(
                "%Y-%m-%d %H:%M:%S")
            csv_dataset.iat[j, 1] = trajectory[j].x
            csv_dataset.iat[j, 2] = trajectory[j].y
        csv_dataset.to_csv(path + 'handle_' + str(i + 1) + '.txt', header=None, index=None)


def process_illinois_data():
    """
    处理 Illinois 数据集 剔除缺失值和不要的列
    :return:
    """
    path = r'E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\data\IllinoisData'
    IllinoisData_files = os.listdir(path)
    IllinoisData_files = IllinoisData_files[:2]
    for person_file in IllinoisData_files:
        date_files = os.listdir(path + "\\" + person_file)
        for date_file in date_files:
            data_files = os.listdir(path + "\\" + person_file + "\\" + date_file)
            for data_file in data_files:
                real_path = path + "\\" + person_file + "\\" + date_file + "\\" + data_file
                df = pd.read_csv(real_path, sep='|', header=None)
                # print(df.isna().sum().sum()) # 统计总共有多少缺失值
                print("删除缺失值前  len = ", len(df))
                if df.isna().sum().sum() != 0:
                    print(df.loc[df.isna().any(1)])  # 显示有缺失值的行
                    df.dropna(inplace=True)  # 将所有含有nan项的row删除
                df[0] = df[3]
                df[1] = df[4]
                df.drop([3, 4], axis=1, inplace=True)
                drop_row = [False for i in range(len(df))]
                pre_x = -1
                pre_y = -1
                for i in range(len(df)):
                    time = df.loc[i, 2]
                    if time is not None and time[1] == '-':
                        hour = 24 + int(time[:3])
                        time = " " + str(hour) + time[3:]
                    if df.loc[i, 0] == pre_x and df.loc[i, 1] == pre_y:
                        drop_row[i] = True
                    elif i > 0 and drop_row[i - 1]:
                        drop_row[i - 1] = False
                    df.loc[i, 2] = ("2021-" + date_file[0:2] + "-" + date_file[2:] + time)
                    pre_x = df.loc[i, 0]
                    pre_y = df.loc[i, 1]
                drop_row[len(df) - 1] = False
                df[3] = drop_row
                df = df[(df[3] == 0)]
                df.drop([3], axis=1, inplace=True)
                df.to_csv(
                    path + "\\processd_data\\" + person_file + "_" + date_file + "_" + data_file[:-3] + "csv",
                    index=None, header=None)
                print("删除缺失值后  len = ", len(df))
                print("输出完成")


if __name__ == '__main__':
    data = pd.read_csv(r"E:\Desktop\Programmer\PythonFile\PythonProject\Experiment\data\WalkingData\6-1-1.txt",sep='\t')
