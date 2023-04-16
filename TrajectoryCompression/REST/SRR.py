from Experiment.TrajectoryCompression.REST.FPA import FPA, filter_sub_reference_track


# Redundant Segment
def SRR(reference_sub_tracks: list, epi_s=500):
    redundant_index = set()
    for i in range(len(reference_sub_tracks)):
        cur_seg = reference_sub_tracks[i].point_list
        for j in range(i + 1, len(reference_sub_tracks)):
            if len(reference_sub_tracks[j].point_list) != len(cur_seg):
                continue
            other_seg = reference_sub_tracks[j].point_list
            d_max = -1
            for k in range(len(cur_seg)):
                d_max = max(cur_seg[k].get_haversine(other_seg[k]), d_max)
            # 说明冗余参考子轨迹
            if d_max <= epi_s:
                redundant_index.add(j)

    res = []
    for i in range(len(reference_sub_tracks)):
        if i in redundant_index:
            continue
        else:
            res.append(reference_sub_tracks[i])
    return res


def get_reference_sub_trajectories(trajectories, epi_s1, min_support_threshold, min_length_threshold):
    rst = FPA(trajectories, epi_s=epi_s1, min_support_threshold=min_support_threshold,
              min_length_threshold=min_length_threshold)
    frst = filter_sub_reference_track(rst)
    frst2 = SRR(frst, epi_s1)
    refect_dict = dict()
    index = 1
    for refe_track in frst2:
        refe_track.id = index
        refect_dict[index] = refe_track
        index += 1
    return frst2, refect_dict
