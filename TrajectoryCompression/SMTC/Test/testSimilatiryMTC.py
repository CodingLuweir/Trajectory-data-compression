import time

import pandas as pd

from typing import List

from Experiment.TrajectoryCompression.SMTC.similarity_based.mtc import mtc_add
from Experiment.common.Point import Point
from Experiment.common.Trajectory import Trajectory
from Experiment.common.zip import zip_compress, zip_decompress
from Experiment.Metrics.metrics import get_PED_error, get_SED_error, get_speed_error, get_angle_error, get_dtw
from Experiment.data.data_process import get_trajectory_dataset


def output_compressed_trajectory(trajectories: List[Trajectory]):
    """
    输出压缩轨迹
    :param trajectories: 压缩后的轨迹集
    :return:
    """
    data = []
    all_len = 0
    for trajectory in trajectories:
        all_len += len(trajectory.points)
        for point in trajectory.points:
            refer_point = point
            while refer_point.p is not None:
                refer_point = refer_point.p
            if point.p is None:
                data.append([int(point.t), round(point.x, 4), round(point.y, 4)])
            else:
                # 映射的时间 保留两位
                # insurance.append([int(point.t), round(refer_point.t, 2)])
                data.append([int(point.t), int(refer_point.t)])
    pd.DataFrame(data).to_csv("mtc_similarity_compressed_trajectory.txt", header=0,
                              index=False)
    print("压缩后总点数：", all_len)



def get_error(trajectories: List[Trajectory], compressed_trajectories: List[Trajectory]) -> list:
    """
    先将压缩轨迹恢复，然后与原始轨迹比较 获得PED误差
    :param trajectories: 原始轨迹
    :param compressed_trajectories: 压缩轨迹
    :return: [平均误差，最大误差]
    """
    average_ped_error = 0
    max_ped_error = 0
    average_sed_error = 0
    max_sed_error = 0
    average_speed_error = 0
    max_speed_error = 0
    average_angle_error = 0
    max_angle_error = 0
    average_dtw_distance = 0
    start_restore_time = time.perf_counter()
    restore_trajectories = []
    for trajectory in compressed_trajectories:
        restore_single_trajectory = []
        reference_trajectory = None
        if trajectory.reference_trajectory_id != -1:
            reference_trajectory = trajectories[trajectory.reference_trajectory_id]
        for i in range(len(trajectory.points)):
            cur_point = trajectory.points[i]
            # 有参考点 根据参考轨迹找到该时间对应的参考位置
            if cur_point.p is not None:
                for j in range(len(reference_trajectory.points) - 1):
                    if reference_trajectory.points[j].t <= cur_point.p.t <= reference_trajectory.points[j + 1].t:
                        real_point = cur_point.p.linear_prediction(reference_trajectory.points[j],
                                                                   reference_trajectory.points[j + 1])
                        break
            while cur_point.p is not None:
                cur_point = cur_point.p
            restore_single_trajectory.append(Point(cur_point.x, cur_point.y, t=trajectory.points[i].t))
        restore_trajectories.append(restore_single_trajectory)
    end_restore_time = time.perf_counter()

    for i in range(len(restore_trajectories)):
        [a, b] = get_PED_error(trajectories[i].points, restore_trajectories[i])
        [c, d] = get_SED_error(trajectories[i].points, restore_trajectories[i])
        [e, f] = get_speed_error(trajectories[i].points, restore_trajectories[i])
        [g, h] = get_angle_error(trajectories[i].points, restore_trajectories[i])
        average_ped_error += a
        max_ped_error = max(max_ped_error, b)
        average_sed_error += c
        max_sed_error = max(max_sed_error, d)
        average_speed_error += e
        max_speed_error = max(max_speed_error, f)
        average_angle_error += g
        max_angle_error = max(max_angle_error, h)
        average_dtw_distance += get_dtw(trajectories[i].points, restore_trajectories[i])
    restore_time = end_restore_time - start_restore_time

    return [average_ped_error, max_ped_error, average_sed_error, max_sed_error,
            average_speed_error, max_speed_error, average_angle_error, max_angle_error, average_dtw_distance,
            restore_time]


def run():
    # dataset_name = "BerlinMOD_0_005_data"
    # dataset_name = "walk_data"
    dataset_name = "airline_data"
    # dataset_name = "illinois_data"
    res = []
    for i in range(75, 90):
        epsilon = 5 * i
        # 加载轨迹 
        trajectories = get_trajectory_dataset(dataset_name)
        compressed_trajectories = []
        compress_start_time = time.perf_counter()
        for trajectory in trajectories:
            mtc_add(trajectory, compressed_trajectories, epsilon)
        compress_end_time = time.perf_counter()
        output_compressed_trajectory(compressed_trajectories)
        # 加载轨迹 
        trajectories = get_trajectory_dataset(dataset_name)
        [a, b, c, d, e, f, g, h, k, l] = get_error(trajectories, compressed_trajectories)
        a /= len(trajectories)
        c /= len(trajectories)
        e /= len(trajectories)
        g /= len(trajectories)
        k /= len(trajectories)

        print("epsilon:", epsilon)
        print("average_ped_error:", a)
        print("max_ped_error:", b)
        print("average_sed_error:", c)
        print("max_sed_error:", d)
        print("average_speed_error:", e)
        print("max_speed_error:", f)
        print("average_angle_error:", g)
        print("max_speed_error:", h)
        print("average_dtw_distance", k)
        print("压缩时间：", (compress_end_time - compress_start_time))
        [m, n] = zip_compress("mtc_similarity_compressed_trajectory.txt")
        decompress_time = zip_decompress("mtc_similarity_compressed_trajectory.zip")

        res.append([epsilon, a, b, c, d,
                    e, f, g, h, k,
                    m, n, (compress_end_time - compress_start_time), decompress_time, l])
    res = pd.DataFrame(res, columns=['误差阈值', '平均ped误差', '最大ped误差', '平均sed误差', '最大sed误差',
                                     '平均速度误差', '最大速度误差', '平均角度误差', '最大角度误差', '平均dtw距离',
                                     '压缩后文件大小', 'zip后文件大小', '压缩时间(s)', '解压时间(s)', '恢复时间(s)'])
    res.to_excel("result" + dataset_name + ".xlsx", index=None)
    return res


if __name__ == '__main__':
    res = run()
