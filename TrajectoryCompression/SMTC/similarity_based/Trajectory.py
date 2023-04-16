class Trajectory:
    traj_id = -1
    refe_traj_id = -1
    points = []

    def __init__(self, traj_id, points: list = None):
        self.traj_id = traj_id
        self.refe_traj_id = -1
        self.points = []
        if points is not None:
            self.points = points

    def set_points(self, points: list):
        self.points = points

    def to_list(self):
        point_list = []
        for ele in self.points:
            point_list.append([ele.t, ele.x, ele.y])
        return point_list
