import numpy as np
from scipy.cluster.hierarchy import linkage, fcluster
from Experiment.common.ReferenceSubTrack import ReferenceSubTrack


def FPA(trajectories, epi_s=500, min_support_threshold=10, min_length_threshold=3):
    """
    训练出参考轨迹集
    :param trajectories:
    :param epi_s: 距离阈值
    :param min_support_threshold:频繁度阈值
    :param min_length_threshold: 最小参考轨迹长度
    :return:
    """

    # 先用聚类 找出计算点 然后将可替换的点标记为计算点
    point_list = []
    data = []
    for trajectory in trajectories:
        for point in trajectory.points:
            point_list.append(point)
            data.append([point.x, point.y])
    data = np.array(data)
    print("共有轨迹点：", len(point_list), "个")

    mergings = linkage(data, method='complete', metric="euclidean")
    cluster_assignments = fcluster(mergings, t=epi_s / 100000, criterion='distance')

    print("所有特征点可分为：", cluster_assignments.max(), " 类")
    temp_cluster_list = [[] for i in range(cluster_assignments.max())]
    for i in range(len(cluster_assignments)):
        temp_cluster_list[cluster_assignments[i] - 1].append(point_list[i])

    index = 1
    for i in range(len(temp_cluster_list)):
        # 符合要求的频繁计算点
        if len(temp_cluster_list[i]) >= min_support_threshold:
            for point in temp_cluster_list[i]:
                point.calculate_index = index
            index += 1

    # 删除同一轨迹中连续的相同计算点
    for trajectory in trajectories:
        point_list = trajectory.points
        new_point_list = [point_list[0]]
        for i in range(1, len(point_list)):
            if point_list[i].calculate_index != -1 \
                    and point_list[i].calculate_index == point_list[i - 1].calculate_index:
                continue
            else:
                new_point_list.append(point_list[i])
        trajectory.points = new_point_list

    # 得到频繁的计算子轨迹
    reference_sub_tracks = []
    for trajectory in trajectories:
        point_list = trajectory.points
        cal_index_list = []
        refe_point_list = []
        if point_list[0].calculate_index != -1:
            cal_index_list.append(point_list[0].calculate_index)
            refe_point_list.append(point_list[0])
        for i in range(1, len(point_list)):
            if point_list[i].calculate_index != -1:
                # 如果能够连接上 就接上
                if point_list[i].calculate_index != point_list[i - 1].calculate_index:
                    refe_point_list.append(point_list[i])
                    cal_index_list.append(point_list[i].calculate_index)
            else:
                # 否则就断了 重新生成参考轨迹
                if len(cal_index_list) >= min_length_threshold:
                    reference_sub_tracks.append(
                        ReferenceSubTrack(ID=-1, trajectory_id=trajectory.trajectory_id, point_list=refe_point_list,
                                          cal_index_list=cal_index_list))
                cal_index_list = []
                refe_point_list = []
        if len(cal_index_list) >= min_length_threshold:
            reference_sub_tracks.append(
                ReferenceSubTrack(ID=-1, trajectory_id=trajectory.trajectory_id, point_list=refe_point_list,
                                  cal_index_list=cal_index_list))
    print("获得频繁的计算子轨迹", len(reference_sub_tracks))
    return reference_sub_tracks


def filter_sub_reference_track(reference_sub_tracks):
    # 频繁计算子轨迹中  每次都要最长的
    frequency_reference_sub_tracks = [reference_sub_tracks[0]]
    for long_track in reference_sub_tracks:
        # 新生成的频繁参考最轨迹
        frequency_reference_sub_tracks2 = []
        add_Flag = True
        for i in range(len(frequency_reference_sub_tracks)):
            # 如果已有的频繁参考子轨迹存在一条是它的子集，那这个已有的就要被删掉
            if set(frequency_reference_sub_tracks[i].cal_index_list).issubset(set(long_track.cal_index_list)):
                continue
            # 如果他是某个已有轨迹的子集，他就不要放进去
            if set(long_track.cal_index_list).issubset(set(frequency_reference_sub_tracks[i].cal_index_list)):
                add_Flag = False
            frequency_reference_sub_tracks2.append(frequency_reference_sub_tracks[i])
        if add_Flag:
            frequency_reference_sub_tracks2.append(long_track)
        frequency_reference_sub_tracks = frequency_reference_sub_tracks2

    print("筛选掉短的，得到长的计算子轨迹", len(frequency_reference_sub_tracks))
    return frequency_reference_sub_tracks


def max_dtw(point_list1: list, point_list2: list):
    n, m = len(point_list1), len(point_list2)
    matrix = []
    for i in range(n):
        matrix.append([0 for j in range(m)])
    matrix[0][0] = point_list1[0].get_haversine(point_list2[0])
    for j in range(1, m):
        matrix[0][j] = max(matrix[0][j - 1], point_list1[0].get_haversine(point_list2[j]))
    for i in range(1, n):
        matrix[i][0] = max(point_list1[i].get_haversine(point_list2[0]), matrix[i - 1][0])
    for i in range(1, n):
        for j in range(1, m):
            d = point_list1[i].get_haversine(point_list2[j])
            matrix[i][j] = max(d, min(matrix[i - 1][j], matrix[i - 1][j - 1], matrix[i][j - 1]))
    return matrix[n - 1][m - 1]
