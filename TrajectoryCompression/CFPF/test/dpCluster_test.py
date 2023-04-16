import time

import pandas as pd

from Experiment.TrajectoryCompression.DP.dp import priority_dp
from Experiment.TrajectoryCompression.CFPF.stcCluster.cluster import cluster_HAC
from Experiment.common.zip import zip_compress, zip_decompress
from Experiment.Metrics import metrics
from Experiment.Metrics.metrics import get_dtw
from Experiment.data.data_process import get_trajectory_dataset


def run_simple():
    # dataset_name = "airline_data"
    dataset_name = "BerlinMOD_0_005_data"
    epsilon = 3.8
    trajectory = get_trajectory_dataset(dataset_name)[0]
    # sample_index = douglas_peucker(trajectory.points, start=0, last=len(trajectory.points) - 1,
    #                                epsilon=epsilon / 100000.0)
    sample_index = priority_dp(trajectory.points, compression_rate=0.8)
    print("剩余点数：" + str(len(sample_index)))
    dp_trajectory = []
    for e in sample_index:
        dp_trajectory.append(trajectory.points[e])
    trajectory.points = dp_trajectory

    origin_trajectories = get_trajectory_dataset(dataset_name)[0]
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

    [a, b] = metrics.get_PED_error(origin_trajectories.points, trajectory.points)
    [c, d] = metrics.get_SED_error(origin_trajectories.points, trajectory.points)
    [e, f] = metrics.get_speed_error(origin_trajectories.points, trajectory.points)
    [g, h] = metrics.get_angle_error(origin_trajectories.points, trajectory.points)
    average_ped_error += a
    max_ped_error = max(max_ped_error, b)
    average_sed_error += c
    max_sed_error = max(max_sed_error, d)
    average_speed_error += e
    max_speed_error = max(max_speed_error, f)
    average_angle_error += g
    max_angle_error = max(max_angle_error, h)
    dtw_distance += get_dtw(origin_trajectories.points, trajectory.points)

    print("epsilon:", epsilon)
    print("average_ped_error:", average_ped_error)
    print("max_ped_error:", max_ped_error)
    print("average_sed_error:", average_sed_error)
    print("max_sed_error:", max_sed_error)
    print("average_speed_error:", average_speed_error)
    print("max_speed_error:", max_speed_error)
    print("average_angle_error:", average_angle_error)
    print("max_angle_error:", max_angle_error)
    print("dtw_distance", dtw_distance)
    print("end")


def run(dataset_name, remain_file_size, origin_size, epsilon=5 / 100000.0):
    for i in range(1, 2):

        trajectories = get_trajectory_dataset(dataset_name)
        dp_start_time = time.perf_counter()
        dp_trajectories = []

        for trajectory in trajectories:
            # dp
            # sample_index = douglas_peucker(trajectory.points, start=0, last=len(trajectory.points) - 1,
            #                                epsilon=epsilon / 100000.0)
            sample_index = priority_dp(trajectory.points, compression_rate=1 - (remain_file_size / origin_size))
            # TD-TR
            # sample_index = td_tr(trajectory.points, start=0, last=len(trajectory.points) - 1,
            #                      epsilon=(epsilon / 100000.0))
            dp_trajectory = []
            for e in sample_index:
                dp_trajectory.append(trajectory.points[e])
            trajectory.points = dp_trajectory
            dp_trajectories.append(dp_trajectory)

        dp_end_time = time.perf_counter()

        cluster_start_time = time.perf_counter()

        cluster_HAC(dp_trajectories, t=epsilon)

        cluster_end_time = time.perf_counter()


        compressed_trajectories = []
        for trajectory in dp_trajectories:
            for ele in trajectory:
                compressed_trajectories.append([int(ele.t), round(ele.x, 4), round(ele.y, 4)])
        pd.DataFrame(compressed_trajectories).to_csv("mtc_dpCluster_test.txt", index=False, header=0)


        origin_trajectories = get_trajectory_dataset(dataset_name)
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

        for i in range(len(dp_trajectories)):
            [a, b] = metrics.get_PED_error(origin_trajectories[i].points, dp_trajectories[i])
            [c, d] = metrics.get_SED_error(origin_trajectories[i].points, dp_trajectories[i])
            [e, f] = metrics.get_speed_error(origin_trajectories[i].points, dp_trajectories[i])
            [g, h] = metrics.get_angle_error(origin_trajectories[i].points, dp_trajectories[i])
            average_ped_error += a
            max_ped_error = max(max_ped_error, b)
            average_sed_error += c
            max_sed_error = max(max_sed_error, d)
            average_speed_error += e
            max_speed_error = max(max_speed_error, f)
            average_angle_error += g
            max_angle_error = max(max_angle_error, h)
            dtw_distance += get_dtw(origin_trajectories[i].points, dp_trajectories[i])

        average_ped_error /= len(origin_trajectories)
        average_sed_error /= len(origin_trajectories)
        average_speed_error /= len(origin_trajectories)
        average_angle_error /= len(origin_trajectories)
        dtw_distance /= len(origin_trajectories)

        print("remain_file_size:", remain_file_size)
        print("epsilon:", epsilon)
        print("average_ped_error:", average_ped_error)
        print("max_ped_error:", max_ped_error)
        print("average_sed_error:", average_sed_error)
        print("max_sed_error:", max_sed_error)
        print("average_speed_error:", average_speed_error)
        print("max_speed_error:", max_speed_error)
        print("average_angle_error:", average_angle_error)
        print("max_angle_error:", max_angle_error)
        print("dtw_distance", dtw_distance)
        print("压缩时间：", (dp_end_time - dp_start_time) + (cluster_end_time - cluster_start_time))
        [a, b] = zip_compress("mtc_dpCluster_test.txt")
        decompress_time = zip_decompress("mtc_dpCluster_test.zip")
        print("压缩率：", origin_size / b)

        global res
        res.append(
            ["" + str(remain_file_size) + " " + str(origin_size) + " " + str(epsilon), average_ped_error, max_ped_error,
             average_sed_error, max_sed_error,
             average_speed_error, max_speed_error, average_angle_error, max_angle_error, dtw_distance,
             a, b, dp_end_time - dp_start_time, cluster_end_time - cluster_start_time, decompress_time, restore_time])


res = []

if __name__ == '__main__':
    # dataset_name = "BerlinMOD_0_005_data"
    # dataset_name = "walk_data"
    # dataset_name = "airline_data"
    dataset_name = "illinois_data"
    origin_size = 763
    remain_file_size = 190
    epsilon = 20 / 100000.0
    run(dataset_name, remain_file_size, origin_size, epsilon)
    res = pd.DataFrame(res, columns=['误差阈值', '平均ped误差', '最大ped误差', '平均sed误差', '最大sed误差',
                                     '平均速度误差', '最大速度误差', '平均角度误差', '最大角度误差', '平均dtw距离',
                                     '压缩后文件大小', 'zip后文件大小', 'DP时间(s)', '聚类时间(s)', '解压时间(s)', '恢复时间'])
    res.to_excel("result" + dataset_name + ".xlsx", index=None)
