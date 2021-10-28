import numpy as np
from numpyFk import numpyFk
import abc


def computeDistances(positions, point):
    return np.linalg.norm(np.add(positions, -point), axis=1)


def computeFks(positions, fks_fun):
    all_fks = []
    for pos in positions.T:
        all_fks.append(np.array(fks_fun(pos)))
    return np.array(all_fks)


class Metric(object):
    def __init__(self, name, measNames, params):
        self._name = name
        self._params = params
        self._measNames = measNames

    @abc.abstractmethod
    def computeMetric(self, data):
        return


class DistanceToPointMetric(Metric):
    def computeMetric(self, data):
        positions = np.stack([data[name] for name in self._measNames])
        point = self._params["point"]
        return computeDistances(positions, point)


class MinimumDistanceToPointMetric(Metric):
    def computeMetric(self, data):
        positions = np.stack([data[name] for name in self._measNames])
        point = self._params["point"]
        distances = computeDistances(positions, point)
        return (np.min(distances), np.argmin(distances))


class TimeToReachGoalMetric(Metric):
    def computeMetric(self, data):
        goal = np.array(self._params["goal"])
        print(goal)
        fks = np.stack([data[name] for name in self._measNames[:-1]]).T
        t = data[self._measNames[-1]]
        des_distance = self._params["des_distance"]
        distances = computeDistances(fks, goal)
        minDistance = np.min(distances)
        print(minDistance)
        indices = np.where(distances < des_distance)
        if indices[0].size == 0:
            return [-1, 0]
        else:
            return [0, float(t[np.min(indices)])]


class ClearanceMetric(Metric):
    def computeMetric(self, data):
        obstacles = self._params["obstacles"]
        m = self._params["m"]
        n = self._params["n"]
        r_body = self._params["r_body"]
        rawData = np.stack([data[name] for name in self._measNames])
        fks = rawData.T.reshape(-1, n, m)
        minDistances = []
        distanceToObsts = {}
        for i, obst in enumerate(obstacles):
            for i_fk in range(0, n):
                distancesToObst = (
                    computeDistances(fks[:, i_fk, :], np.array(obst.x()))
                    - obst.r()
                    - r_body
                )
                minDistToObst = float(np.min(distancesToObst))
                minDistances.append(minDistToObst)
                distanceToObsts["obst" + str(i) + "_fk" + str(i_fk)] = {
                    "dist": minDistToObst,
                    "loc": obst.toArray()[0:m].tolist(),
                    "r": obst.r(),
                }
        return {"minDist": float(min(minDistances)), "allMinDist": distanceToObsts}


class SelfClearanceMetric(Metric):
    def computeMetric(self, data):
        m = self._params["m"]
        n = self._params["n"]
        r_body = self._params["r_body"]
        rawData = np.stack([data[name] for name in self._measNames])
        fks = rawData.T.reshape(-1, n+1, m)
        minDistances = []
        distanceToBodies = {}
        for i_fk in range(n+1):
            for j_fk in range(i_fk + 2, n+1):
                distances = (
                    computeDistances(fks[:, i_fk, :], fks[:, j_fk, :]) - 2 * r_body
                )
                minDistance = float(np.min(distances))
                minDistances.append(minDistance)
                distanceToBodies[str(i_fk) + "_" + str(j_fk)] = {"dist": minDistance}
        return {"minDist": min(minDistances), "allMinDist": distanceToBodies}


class SolverTimesMetric(Metric):
    def computeMetric(self, data):
        interval = self._params["interval"]
        t_planning = data[self._measNames[0]]
        if interval == 1:
            return float(np.mean(t_planning))
        else:
            return float(
                np.mean(t_planning[: -(interval - 1)].reshape(-1, interval), axis=0)[0]
            )


class PathLengthMetric(Metric):
    def computeMetric(self, data):
        pathLength = 0
        fks = np.stack([data[name] for name in self._measNames]).T
        pathLength = [np.linalg.norm(fks[i] - fks[i - 1]) for i in range(1, len(fks))]
        return float(np.sum(pathLength))


class SuccessMetric(Metric):
    def computeMetric(self, data):
        minClearance = self._params["minClearance"]
        reachingFlag = self._params["reachingFlag"]
        if reachingFlag < 0:
            return [False, -2]
        if minClearance < 0:
            return [False, -1]
        return [True, 0]
