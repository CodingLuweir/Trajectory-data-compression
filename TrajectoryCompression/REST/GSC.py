# Matchable Reference Trajectory
import time
import pandas as pd
from Experiment.Metrics import metrics
from Experiment.Metrics.metrics import get_dtw
from Experiment.TrajectoryCompression.REST.FPA import max_dtw
from Experiment.TrajectoryCompression.REST.SRR import get_reference_sub_trajectories
from Experiment.common.Point import Point
from Experiment.common.zip import zip_compress, zip_decompress
from Experiment.data.data_process import get_trajectory_dataset


def MRT(frst, trajectories, epi_s=500):
    compressed_trajectories = []
    restore_trajectories = []
    for trajectory in trajectories:
        point_list = trajectory.points
        compressed_point_list = []
        restore_point_list = []
        i = 0
        matchable_count = 0
        while i < len(point_list):
            matchable = False
            min_maxdtw = 99999999
            min_maxdtw_index = -1
            for j in range(len(frst)):
                refe_sub_track = frst[j].point_list
                if i + len(refe_sub_track) >= len(point_list):
                    continue
                # 如果匹配上了
                maxdtw = max_dtw(point_list[i:i + len(refe_sub_track)], refe_sub_track)
                if maxdtw <= epi_s and maxdtw < min_maxdtw:
                    min_maxdtw = maxdtw
                    min_maxdtw_index = j
                    matchable = True
            if matchable is True and min_maxdtw_index != -1:
                print("轨迹点", i, "匹配参考轨迹段", min_maxdtw_index)
                compressed_point_list.append(frst[min_maxdtw_index].id)
                tar_refe_track = frst[min_maxdtw_index].point_list
                j = i + len(tar_refe_track)
                while j < len(point_list) and max_dtw(point_list[i:j], tar_refe_track) <= epi_s:
                    j += 1
                factor = int((point_list[j - 1].t - point_list[i].t) / (len(tar_refe_track)))
                for k in range(0, len(tar_refe_track)):
                    restore_point_list.append(
                        Point(t=point_list[i].t + factor * k, x=tar_refe_track[k].x, y=tar_refe_track[k].y))

                i = j
                matchable_count += 1
            else:
                compressed_point_list.append(point_list[i])
                restore_point_list.append(point_list[i])
                i += 1
        restore_trajectories.append(restore_point_list)
        compressed_trajectories.append(compressed_point_list)
        print("第", trajectory.trajectory_id, "条轨迹匹配" + str(matchable_count) + "个参考段")
    return compressed_trajectories, restore_trajectories


res = []


def run(frst, refect_dict, trajectories, dataset_name, num, epi_s2=500):
    # 压缩
    compress_start_time = time.perf_counter()
    compressed_trajectories, restore_trajectories = MRT(frst, trajectories[:num], epi_s=epi_s2)
    compress_end_time = time.perf_counter()

    output = []
    for traj in compressed_trajectories:
        for ele in traj:
            if not isinstance(ele, int):
                output.append([int(ele.t), round(ele.x, 4), round(ele.y, 4)])
    pd.DataFrame(output).to_csv("REST.txt", index=False, header=0)

    # 分析误差
    # 加载数据

    origin_trajectories = get_trajectory_dataset(dataset_name, num=num)
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

    for i in range(len(origin_trajectories)):
        # 恢复
        restore_start_time = time.perf_counter()
        restore_point_list = []
        for ele in compressed_trajectories[i]:
            if isinstance(ele, int):
                for point in refect_dict[ele].point_list:
                    restore_point_list.append([point.t, point.x, point.y])
            else:
                restore_point_list.append([ele.t, ele.x, ele.y])
        restore_end_time = time.perf_counter()

        [a, b] = metrics.get_PED_error(origin_trajectories[i].points, restore_trajectories[i])
        [c, d] = metrics.get_SED_error(origin_trajectories[i].points, restore_trajectories[i])
        [e, f] = metrics.get_speed_error(origin_trajectories[i].points, restore_trajectories[i])
        [g, h] = metrics.get_angle_error(origin_trajectories[i].points, restore_trajectories[i])
        average_ped_error += a
        max_ped_error = max(max_ped_error, b)
        average_sed_error += c
        max_sed_error = max(max_sed_error, d)
        average_speed_error += e
        max_speed_error = max(max_speed_error, f)
        average_angle_error += g
        max_angle_error = max(max_angle_error, h)
        dtw_distance += get_dtw(origin_trajectories[i].points, restore_trajectories[i])
        restore_time += (restore_end_time - restore_start_time)

    average_ped_error /= len(origin_trajectories)
    average_sed_error /= len(origin_trajectories)
    average_speed_error /= len(origin_trajectories)
    average_angle_error /= len(origin_trajectories)
    dtw_distance /= len(origin_trajectories)
    restore_time /= len(origin_trajectories)
    compress_time = (compress_end_time - compress_start_time) / len(origin_trajectories)

    print("epsilon:", epi_s2)
    print("average_ped_error:", average_ped_error)
    print("max_ped_error:", max_ped_error)
    print("average_sed_error:", average_sed_error)
    print("max_sed_error:", max_sed_error)
    print("average_speed_error:", average_speed_error)
    print("max_speed_error:", max_speed_error)
    print("average_angle_error:", average_angle_error)
    print("max_angle_error:", max_angle_error)
    print("dtw_distance", dtw_distance)
    print("压缩时间：", compress_time)
    print("恢复时间：", restore_time)
    [a, b] = zip_compress("REST.txt")
    decompress_time = zip_decompress("REST.zip")
    res.append(
        ["" + str(epi_s1) + " " + str(min_support_threshold) + " " + str(min_length_threshold) + " " + str(epi_s2),
         average_ped_error, max_ped_error, average_sed_error, max_sed_error,
         average_speed_error, max_speed_error, average_angle_error, max_angle_error, dtw_distance,
         a, b, compress_time, decompress_time, restore_time])
    return b


if __name__ == '__main__':
    # dataset_name = "BerlinMOD_0_005_data"
    dataset_name = "walk_data"
    # dataset_name = "airline_data"
    # dataset_name = "illinois_data"
    trajectories = get_trajectory_dataset(dataset_name=dataset_name, num=9)
    # 计算点
    min_support_threshold = 3
    min_length_threshold = 3
    epi_s1 = 10
    # 构造参考轨迹
    frst, refect_dict = get_reference_sub_trajectories(trajectories, epi_s1, min_support_threshold,
                                                       min_length_threshold)
    epi_s2 = 10
    origin_size = 56.5
    num = 9
    remain_size = run(frst, refect_dict, trajectories, dataset_name, num=num, epi_s2=epi_s2)
    print(remain_size)
    while remain_size >= 0.56:
        remain_size = run(frst, refect_dict, trajectories, dataset_name, num=num, epi_s2=epi_s2)
        print("remain_size", remain_size)
        epi_s2 += 5
    res = pd.DataFrame(res, columns=['误差阈值', '平均ped误差', '最大ped误差', '平均sed误差', '最大sed误差',
                                     '平均速度误差', '最大速度误差', '平均角度误差', '最大角度误差', '平均dtw距离',
                                     '压缩后文件大小', 'zip后文件大小', '压缩时间(s)', '解压时间(s)', '恢复时间'])
    res.to_excel("result" + dataset_name + ".xlsx", index=None)
