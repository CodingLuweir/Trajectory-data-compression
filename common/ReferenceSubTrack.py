class ReferenceSubTrack(object):
    def __init__(self, ID=-1, trajectory_id=-1, point_list: list = None, cal_index_list: list = None):
        self.id = ID
        self.trajectory_id = trajectory_id
        self.point_list = point_list
        self.cal_index_list = cal_index_list
