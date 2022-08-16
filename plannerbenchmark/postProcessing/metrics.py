import numpy as np
from abc import ABC, abstractmethod
from typing import Callable


def computeDistances(points_a: np.ndarray, points_b: np.ndarray):
    """Compute the distance between two array of points."""
    return np.linalg.norm(np.add(points_a, -points_b), axis=1)


class Metric(ABC):
    """Abstract metric to assess the performance of the motion planner.

    Attributes
    ----------

    _name: str
        Name of the metric.
    _params: dict
        Additional information and data is passed through the dictonary.
    _measNames: list
        List of keys that are needed from the results.

    """

    def __init__(self, name: str, measNames: list, params: dict):
        self._name: str = name
        self._params: dict = params
        self._measNames: list = measNames

    @abstractmethod
    def computeMetric(self, data: dict):
        return

    def name(self):
        return self._name


class DistanceToPointMetric(Metric):
    """Metric to compute the distance to a point.

    Requires the coordinates of the point.
    """

    def computeMetric(self, data):
        positions = np.stack([data[name] for name in self._measNames])
        point = self._params["point"]
        return computeDistances(positions, point)


class MinimumDistanceToPointMetric(Metric):
    """Metric to compute the minimum distance to a point.

    Requires the coordinates of the point.
    """

    def computeMetric(self, data):
        positions = np.stack([data[name] for name in self._measNames])
        point = self._params["point"]
        distances = computeDistances(positions, point)
        return (np.min(distances), np.argmin(distances))


class TimeToReachGoalMetric(Metric):
    """Metric to compute the time it took to reach the goal.

    Requires the threshold, `des_distance`.
    """

    def computeMetric(self, data):
        dimension_goal = len(self._params["goal_indices"])
        fks = np.stack([data[name] for name in self._measNames[:dimension_goal]]).T
        goal = np.stack([data[name] for name in self._measNames[dimension_goal:-1]]).T
        time_steps= data[self._measNames[-1]]
        des_distance = self._params["des_distance"]
        distances = computeDistances(fks, goal)
        indices = np.where(distances < des_distance)
        if indices[0].size == 0:
            return {"short": -1, "flag": -1}
        else:
            return {"short": float(time_steps[np.min(indices)]), "flag": 0}


class IntegratedErrorMetric(Metric):
    """Metric to compute the integrated deviation error from reference trajectory.

    Requires the threshold, `des_distance`.
    This metric computes the averaged deviation error from the first time the
    threshold was reached.
    """

    def computeMetric(self, data):
        m = self._params["dimension_obstacle"]
        des_distance = self._params["des_distance"]
        fks = np.stack([data[name] for name in self._measNames[:m]]).T
        goal = np.stack([data[name] for name in self._measNames[m:-1]]).T
        time_steps = np.array(data[self._measNames[-1]])
        distances = computeDistances(fks, goal)
        indices = np.where(distances < des_distance)
        if indices[0].size == 0:
            return {"short": 1000}
        else:
            # trackingTime = float((time_steps[-1] - time_steps[indices[0][0]]))
            # trackingError = float(np.sum(distances[indices[0][0]:]))/trackingTime
            trackingError = np.average(distances[indices[0][0] :])
            return {"short": float(trackingError)}


class ClearanceMetric(Metric):
    """Metric to compute the minimum clearance from any obstacle.

    Requires the dimension of obstacles, `m`, the dimension
    of the configuration space, `n`,
    all obstacles present in the scenario, `obstacles` and
    the link inflation `r_body`.
    Based on the forward kinematics, the distance between all links and all
    obstacles is computed. The clearance is the minumum of all those values.
    The output of this metric provides information about the minimum
    distance between every link and every obstacle.

    """

    def computeMetric(self, data):
        obstacles = self._params["obstacles"]
        m = obstacles[0].dim()
        n = self._params["n"]
        r_body = self._params["r_body"]
        rawData = np.stack([data[name] for name in self._measNames])
        fks = rawData.T.reshape(-1, n, m)
        minDistances = []
        distanceToObsts = {}
        for i, obst in enumerate(obstacles):
            for i_fk in range(0, n):
                distancesToObst = (
                    computeDistances(fks[:, i_fk, :], np.array(obst.position()))
                    - obst.radius()
                    - r_body
                )
                minDistToObst = float(np.min(distancesToObst)) + 1e-3  # make  MPC happy
                minDistances.append(minDistToObst)
                distanceToObsts["obst" + str(i) + "_fk" + str(i_fk)] = {
                    "dist": minDistToObst,
                    "loc": list(obst.position()),
                    "r": obst.radius(),
                }
        return {"short": float(min(minDistances)), "allMinDist": distanceToObsts}

class InverseClearanceMetric(ClearanceMetric):

    """Metric to compute the inverse of the minimum clearance from any obstacle.

    InverseClearance is the inverse of ClearanceMetric.
    """

    def computeMetric(self, data):
        evaluation = super().computeMetric(data)
        evaluation['short'] = 1.0 / evaluation['short']
        return evaluation


class DynamicClearanceMetric(Metric):
    """Metric to compute the clearance with dynamic obstacles.

    Requires the dimension of obstacles, `m`, the dimension
    of the configuration space, `n`,
    the inflation radius of the robot links, `r_body` and the radius
    of the dynamic obstacles, `r_obsts`. Minimum distances between all
    robot links and all obstacles are computed and the returned.
    """

class InverseDynamicClearanceMetric(ClearanceMetric):

    """Metric to compute the inverse of the minimum clearance from any obstacle.

    InverseClearance is the inverse of ClearanceMetric.
    """

    def computeMetric(self, data):
        evaluation = super().computeMetric(data)
        evaluation['short'] = 1.0 / evaluation['short']
        return evaluation

    def computeMetric(self, data):
        m = self._params["dimension_obstacle"]
        n = self._params["n"]
        r_body = self._params["r_body"]
        r_obsts = self._params["r_obsts"]
        rawData = np.stack([data[name] for name in self._measNames[: m * n]])
        fks = rawData.T.reshape(-1, n, m)
        nb_obst = len(r_obsts)
        obsts = []
        for i in range(nb_obst):
            obstNames = [f"obst_{i}_{j}_0" for j in range(m)]
            obsts.append(np.stack([data[name] for name in obstNames]).T)
        time_steps = np.array(data[self._measNames[-1]])
        minDistances = []
        distanceToObsts = {}
        for i, obst in enumerate(obsts):
            for i_fk in range(0, n):
                distancesToObst = (
                    computeDistances(fks[:, i_fk, :], obst) - r_body - r_obsts[i]
                )
                index = np.argmin(distancesToObst)
                minDistToObst = float(min(distancesToObst)) + 1e-3  # make MPC happy
                minDistances.append(minDistToObst)
                distanceToObsts["obst" + str(i) + "_fk" + str(i_fk)] = {
                    "dist": minDistToObst,
                    "loc": obst[index].tolist(),
                    "robotLoc": fks[index, i_fk, :].tolist(),
                    "r_body": r_body,
                    "r_obst": r_obsts[i],
                }
        return {"short": 1.0/float(min(minDistances)), "allMinDist": distanceToObsts}


class SelfClearanceMetric(Metric):
    """Metric to compute clearance between different links on the robot.

    Requires the dimension of obstacles, `m`, the dimension of the
    configuration space, `n`, the inflation radius of the hrobot links,
    `r_body` and the list of pairs that should be evaluated, `pairs`.
    """

    def computeMetric(self, data):
        m = self._params["dimension_obstacle"]
        n = self._params["n"]
        r_body = self._params["r_body"]
        pairs = self._params["pairs"]
        rawData = np.stack([data[name] for name in self._measNames])
        fks = rawData.T.reshape(-1, n + 1, m)
        minDistances = []
        distanceToBodies = {}
        for pair in pairs:
            i_fk = pair[0]
            j_fk = pair[1]
            distances = computeDistances(fks[:, i_fk, :], fks[:, j_fk, :]) - 2 * r_body
            minDistance = float(np.min(distances))
            minDistances.append(minDistance)
            distanceToBodies[str(i_fk) + "_" + str(j_fk)] = {"dist": minDistance}
        return {"short": min(minDistances), "allMinDist": distanceToBodies}


class SolverTimesMetric(Metric):
    """Metric to compute the average solver time of the motion planner.

    Requires the interval at which the planner was invoked.
    Computes the solvertime in miliseconds.
    """

    def computeMetric(self, data):
        interval = self._params["interval"]
        t_planning = data[self._measNames[0]]
        return {"short": 1000 * float(np.mean(t_planning[0::interval]))}


class PathLengthMetric(Metric):
    """Metric to compute the pathlength of the trajectory in workspace.

    The path length is computed in the work space and not the configuration space.
    """

    def computeMetric(self, data):
        pathLength = 0
        fks = np.stack([data[name] for name in self._measNames]).T
        pathLength = [np.linalg.norm(fks[i] - fks[i - 1]) for i in range(1, len(fks))]
        return {"short": float(np.sum(pathLength))}


class SuccessMetric(Metric):
    """Metric to compute if the experiment was successful.

    Requires minimum clearance and information whether the goal
    was reached. Both can be computed using one of the above metrics.
    An experiment is considered sucessful if the goal was reached
    and the clearance was always positive.
    """

    def computeMetric(self, data):
        minClearance = self._params["minClearance"]
        reachingFlag = self._params["reachingFlag"]
        if reachingFlag < 0:
            return {"short": -2}
        if minClearance < 0:
            return {"short": -1}
        return {"short": 1}
