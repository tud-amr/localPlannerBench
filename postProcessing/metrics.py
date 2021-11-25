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
        m = self._params['m']
        fks = np.stack([data[name] for name in self._measNames[:m]]).T
        goal = np.stack([data[name] for name in self._measNames[m:-1]]).T
        t = data[self._measNames[-1]]
        des_distance = self._params["des_distance"]
        distances = computeDistances(fks, goal)
        minDistance = np.min(distances)
        indices = np.where(distances < des_distance)
        if indices[0].size == 0:
            return {'short': -1, 'flag': -1}
        else:
            return {'short': float(t[np.min(indices)]), 'flag': 0}


class IntegratedErrorMetric(Metric):
    def computeMetric(self, data):
        m = self._params['m']
        des_distance = self._params['des_distance']
        fks = np.stack([data[name] for name in self._measNames[:m]]).T
        goal = np.stack([data[name] for name in self._measNames[m:-1]]).T
        t = np.array(data[self._measNames[-1]])
        distances = computeDistances(fks, goal)
        indices = np.where(distances < des_distance)
        if indices[0].size == 0:
            return {'short': 1000}
        else:
            trackingTime = float((t[-1] - t[indices[0][0]]))
            trackingError = float(np.sum(distances[indices[0][0]:]))/trackingTime
            return {'short': trackingError}


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
        return {"short": float(min(minDistances)), "allMinDist": distanceToObsts}


class DynamicClearanceMetric(Metric):
    def computeMetric(self, data):
        m = self._params['m']
        n = self._params['n']
        r_body = self._params['r_body']
        r_obst = self._params['r_obst']
        rawData = np.stack([data[name] for name in self._measNames[:m*n]])
        fks = rawData.T.reshape(-1, n, m)
        #fks = np.stack([data[name] for name in self._measNames[:m]]).T
        obst = np.stack([data[name] for name in self._measNames[m*n:m*n+m]]).T
        t = np.array(data[self._measNames[-1]])
        minDistances = []
        distanceToObsts = {}
        for i in range(1):
            for i_fk in range(0, n):
                distancesToObst = computeDistances(fks[:, i_fk, :], obst) - r_body - r_obst
                index = np.argmin(distancesToObst)
                minDistToObst = float(min(distancesToObst)) + 1e-3 # make MPC happy
                minDistances.append(minDistToObst)
                distanceToObsts["obst" + str(i) + "_fk" + str(i_fk)] = {
                    "dist": minDistToObst,
                    "loc": obst[index].tolist(),
                    "robotLoc": fks[index, i_fk, :].tolist(),
                    "r": r_body,
                }
        return {
            "short": float(min(minDistances)),
            "allMinDist": distanceToObsts
        }


class SelfClearanceMetric(Metric):
    def computeMetric(self, data):
        m = self._params["m"]
        n = self._params["n"]
        r_body = self._params["r_body"]
        pairs = self._params["pairs"]
        rawData = np.stack([data[name] for name in self._measNames])
        fks = rawData.T.reshape(-1, n+1, m)
        minDistances = []
        distanceToBodies = {}
        for pair in pairs:
            i_fk = pair[0]
            j_fk = pair[1]
            distances = (
                computeDistances(fks[:, i_fk, :], fks[:, j_fk, :]) - 2 * r_body
            )
            minDistance = float(np.min(distances))
            minDistances.append(minDistance)
            distanceToBodies[str(i_fk) + "_" + str(j_fk)] = {"dist": minDistance}
        return {"short": min(minDistances), "allMinDist": distanceToBodies}


class SolverTimesMetric(Metric):
    def computeMetric(self, data):
        interval = self._params["interval"]
        t_planning = data[self._measNames[0]]
        return {'short': float(np.mean(t_planning[0::interval]))}


class PathLengthMetric(Metric):
    def computeMetric(self, data):
        pathLength = 0
        fks = np.stack([data[name] for name in self._measNames]).T
        pathLength = [np.linalg.norm(fks[i] - fks[i - 1]) for i in range(1, len(fks))]
        return {'short': float(np.sum(pathLength))}


class SuccessMetric(Metric):
    def computeMetric(self, data):
        minClearance = self._params["minClearance"]
        reachingFlag = self._params["reachingFlag"]
        if reachingFlag < 0:
            return {'short': False, 'flag': -2}
        if minClearance < 0:
            return {'short': False, 'flag': -1}
        return {'short': True, 'flag': 0}
