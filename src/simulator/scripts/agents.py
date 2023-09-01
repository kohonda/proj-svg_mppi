import numpy as np
import csv
from agent_utils import (
    get_actuation,
    nearest_point_on_trajectory_py2,
    first_point_on_trajectory_intersecting_circle,
)


class Agent(object):
    def __init__(self, csv_path):
        # TODO: load waypoints from csv
        self.waypoints = None
        self.safe_speed = 0.5

    def plan(self, obs):
        pass


class StaticAgent(Agent):
    def __init__(self, csv_path, seed=0):
        super().__init__(csv_path)

        self._rng = np.random.default_rng(seed)

        with open(csv_path) as f:
            wpts = [tuple(line) for line in csv.reader(f)]
            self.waypoints = np.array(
                [
                    (
                        float(pt[0]),
                        float(pt[1]),
                        float(pt[2]),
                        float(pt[3]),
                        float(pt[4]),
                        float(pt[5]),
                    )
                    for pt in wpts
                ]
            )

    def get_random_pos(self, radius) -> np.ndarray:
        # Select a random waypoint
        waypoint = self._rng.choice(self.waypoints)
        x = waypoint[0]
        y = waypoint[1]
        # Select a random point within a circle of radius `radius` centered at the waypoint
        rand_r = self._rng.uniform(0, radius)
        rand_theta = self._rng.uniform(-np.pi, np.pi)
        rand_x = x + rand_r * np.cos(rand_theta)
        rand_y = y + rand_r * np.sin(rand_theta)
        rand_yaw = self._rng.uniform(-np.pi, np.pi)

        return np.array([[rand_x, rand_y, rand_yaw]])

    def plan(self, obs):
        return 0.0, 0.0


class PurePursuitAgent(Agent):
    def __init__(self, csv_path, wheelbase):
        super(PurePursuitAgent, self).__init__(csv_path)
        self.lookahead_distance = 1.0
        self.wheelbase = wheelbase
        self.max_reacquire = 10.0
        with open(csv_path) as f:
            wpts = [tuple(line) for line in csv.reader(f)]
            self.waypoints = np.array(
                [
                    (
                        float(pt[0]),
                        float(pt[1]),
                        float(pt[2]),
                        float(pt[3]),
                        float(pt[4]),
                        float(pt[5]),
                    )
                    for pt in wpts
                ]
            )

    def _get_current_waypoint(self, waypoints, lookahead_distance, position, theta):
        wpts = waypoints[:, 0:2]
        nearest_point, nearest_dist, t, i = nearest_point_on_trajectory_py2(
            position, wpts
        )
        if nearest_dist < lookahead_distance:
            lookahead_point, i2, t2 = first_point_on_trajectory_intersecting_circle(
                position, lookahead_distance, wpts, i + t, wrap=True
            )
            if i2 == None:
                return None
            current_waypoint = np.empty(waypoints[i2, :].shape)
            # x, y
            current_waypoint[0:2] = waypoints[i2, 0:2]
            # theta
            current_waypoint[3] = waypoints[i2, 3]
            # speed
            current_waypoint[2] = waypoints[i2, 2]
            return current_waypoint
        elif nearest_dist < self.max_reacquire:
            return waypoints[i, :]
        else:
            return None

    def plan(self, obs):
        pose_x = obs["poses_x"][1]
        pose_y = obs["poses_y"][1]
        pose_theta = obs["poses_theta"][1]
        position = np.array([pose_x, pose_y])
        lookahead_point = self._get_current_waypoint(
            self.waypoints, self.lookahead_distance, position, pose_theta
        )
        if lookahead_point is None:
            return self.safe_speed, 0.0
        speed, steering_angle = get_actuation(
            pose_theta,
            lookahead_point,
            position,
            self.lookahead_distance,
            self.wheelbase,
        )
        return speed, steering_angle
