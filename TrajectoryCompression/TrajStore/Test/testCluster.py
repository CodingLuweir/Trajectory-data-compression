import time
import pandas as pd
from typing import List
from Experiment.TrajectoryCompression.TrajStore.src.cluster import traj_store_cluster
from Experiment.common.Point import Point
from Experiment.common.Trajectory import Trajectory
from Experiment.common.zip import zip_compress, zip_decompress
from Experiment.Metrics.metrics import get_PED_error, get_SED_error, get_speed_error, get_angle_error, get_dtw
from Experiment.data.data_process import get_trajectory_dataset


def output_origin_trajectory(trajectories):
    for trajectory in trajectories:
        data = []
        start_time = trajectory.points[0].t
        for point in trajectory.points:
            data.append([point.t - start_time, round(point.x, 4), round(point.y, 4)])
        pd.DataFrame(data).to_csv("output_origin_trajectory_" + str(trajectory.trajectory_id) + ".csv", header=False,
                                  index=False)


def output_compressed_trajectory(trajectories):
    data = []
    for trajectory in trajectories:
        # 自身就是本聚类里面的参考轨迹
        if trajectory.reference_trajectory_id == -1:
            for point in trajectory.points:
                data.append([int(point.t), round(point.x, 4), round(point.y, 4)])
        # 有参考轨迹 就只用存映射时间
        else:
            data += [[ele] for ele in trajectory.reference_time]
    pd.DataFrame(data).to_csv("output_compressed_trajectory.txt", header=False,
                              index=False)


def get_restore_trajectory(compressed_trajectory: Trajectory, hash_map):

    if compressed_trajectory.reference_trajectory_id == -1:
        return compressed_trajectory
    
    reference_trajectory = hash_map[compressed_trajectory.reference_trajectory_id]
    restore_points = []

    for i in range(len(compressed_trajectory.reference_time)):
        restore_points.append(Point(x=reference_trajectory.points[i].x, y=reference_trajectory.points[i].y,
                                    t=int(compressed_trajectory.reference_time[i])))
    return Trajectory(compressed_trajectory.trajectory_id, restore_points)


def linear_eliminate(trajectories: List[Trajectory], epsilon: float):
    """
    对 trajectories 进行线性消除无关点
    :param trajectories: 轨迹集
    :param epsilon: 欧氏距离误差 
    :return: 返回进行线性消除后的轨迹集
    """
    linear_eliminate_trajectories = []
    for trajectory in trajectories:
        new_trajectory_points = []
        new_trajectory_points.append(trajectory.points[0])
        first_index = 0
        last_index = first_index + 1
        while last_index < len(trajectory.points):
            first_point = trajectory.points[first_index]
            last_point = trajectory.points[last_index]
            flag = True
            for mid_index in range(first_index + 1, last_index):
                mid_point = trajectory.points[mid_index]
                temp_point = mid_point.linear_prediction(first_point, last_point)
                if mid_point.get_haversine(temp_point) > epsilon:
                    flag = False
                    break
            if not flag or last_index == len(trajectory.points) - 1:
                new_trajectory_points.append(trajectory.points[last_index - 1])
                first_index = last_index - 1
            last_index += 1
        # 加入最后一个点
        new_trajectory_points.append(trajectory.points[-1])
        linear_eliminate_trajectories.append(Trajectory(trajectory.trajectory_id, new_trajectory_points))
    return linear_eliminate_trajectories



def run():
    # dataset_name = "BerlinMOD_0_005_data"
    # dataset_name = "walk_data"
    # dataset_name = "airline_data"
    dataset_name = "illinois_data"
    res = []
    for i in range(200, 400):
        # 指标
        average_ped_error = 0
        max_ped_error = 0
        average_sed_error = 0
        max_sed_error = 0
        average_speed_error = 0
        max_speed_error = 0
        average_angle_error = 0
        max_angle_error = 0
        dtw_distance = 0
        restore_time = 0
        epsilon = i * 5

        # 加载数据集
        trajectories = get_trajectory_dataset(dataset_name)
        compress_start_time = time.perf_counter()

        # 第一部分 线性法消除无关点  
        linear_eliminate_trajectories = linear_eliminate(trajectories, 0.5 * epsilon)

        # 第二部分 聚类压缩  
        group = traj_store_cluster(linear_eliminate_trajectories, 0.25 * epsilon)
        compress_end_time = time.perf_counter()
        output_compressed_trajectory(linear_eliminate_trajectories)

        # 第三部分 轨迹恢复和误差测量
        hash_map = {}
        for trajectory in linear_eliminate_trajectories:
            if trajectory.reference_trajectory_id == -1:
                hash_map[trajectory.trajectory_id] = trajectory
        for i in range(len(linear_eliminate_trajectories)):
            # 恢复数据
            restore_start_time = time.perf_counter()
            restore_trajectory = get_restore_trajectory(linear_eliminate_trajectories[i], hash_map)
            restore_end_time = time.perf_counter()
            restore_time += (restore_end_time - restore_start_time)

            [a, b] = get_PED_error(trajectories[i].points, restore_trajectory.points)
            [c, d] = get_SED_error(trajectories[i].points, restore_trajectory.points)
            [e, f] = get_speed_error(trajectories[i].points, restore_trajectory.points)
            [g, h] = get_angle_error(trajectories[i].points, restore_trajectory.points)
            average_ped_error += a
            max_ped_error = max(max_ped_error, b)
            average_sed_error += c
            max_sed_error = max(max_sed_error, d)
            average_speed_error += e
            max_speed_error = max(max_speed_error, f)
            average_angle_error += g
            max_angle_error = max(max_angle_error, h)
            dtw_distance += get_dtw(trajectories[i].points, restore_trajectory.points)

        average_ped_error /= len(trajectories)
        average_sed_error /= len(trajectories)
        average_speed_error /= len(trajectories)
        average_angle_error /= len(trajectories)
        dtw_distance /= len(trajectories)

        print("距离阈值:", epsilon)
        print("average_ped_error:", average_ped_error)
        print("max_ped_error:", max_ped_error)
        print("average_sed_error:", average_sed_error)
        print("max_sed_error:", max_sed_error)
        print("average_speed_error:", average_speed_error)
        print("max_speed_error:", max_speed_error)
        print("average_angle_error:", average_angle_error)
        print("max_angle_error:", max_angle_error)
        print("average_dtw_distance", dtw_distance)
        print("压缩时间：", (compress_end_time - compress_start_time))
        print("restore_time", restore_time)
        [a, b] = zip_compress("output_compressed_trajectory.txt")
        decompress_time = zip_decompress("output_compressed_trajectory.zip")

        res.append(
            [epsilon, average_ped_error, max_ped_error, average_sed_error, max_sed_error,
             average_speed_error, max_speed_error, average_angle_error, max_angle_error, dtw_distance,
             a, b, (compress_end_time - compress_start_time), decompress_time, restore_time])
    res = pd.DataFrame(res, columns=['误差阈值', '平均ped误差', '最大ped误差', '平均sed误差', '最大sed误差', '平均速度误差', '最大速度误差', '平均角度误差',
                                     '最大角度误差', '平均dtw距离', '压缩后文件大小', 'zip后文件大小', '压缩时间(s)', '解压时间(s)', '恢复时间(s)'])
    res.to_excel("result" + dataset_name + ".xlsx", index=None)
    return res


if __name__ == '__main__':
    res = run()
