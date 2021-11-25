import numpy as np
import casadi as ca

from fabricsExperiments.infrastructure.variables import t
from optFabrics.diffGeometry.referenceTrajectory import AnalyticTrajectory
from optFabrics.diffGeometry.splineTrajectory import SplineTrajectory


class SubGoalIncompleteError(Exception):
    pass


class SubGoalMissmatchDimensionError(Exception):
    pass


class SubGoal(object):
    def __init__(self, name, goalDict):
        self._name = name
        self._required_keys = [
            "m",
            "w",
            "prime",
            "indices",
            "parent_link",
            "child_link",
            "desired_position",
            "type"
        ]
        self._randomize_keys = ["low", "high"]
        self._setup = goalDict
        self.checkCompleteness()
        self.checkDimensionality()
        if self.type() == 'static':
            self.initializeStaticGoal()
            self._dynamic = False
        elif self.type() == 'analytic':
            self.initializeAnalyticTrajectory()
            self._dynamic = True
        elif self.type() == 'spline':
            self.initializeSplineTrajectory()
            self._dynamic = True

    def checkDimensionality(self):
        if self.m() != len(self.desiredPosition()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between goal and m"
            )
        if self.m() != len(self.indices()):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between indices and m"
            )

    def checkCompleteness(self):
        incomplete = False
        missingKeys = ""
        for key in self._required_keys:
            if key not in self._setup.keys():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise SubGoalIncompleteError("Missing keys: %s" % missingKeys[:-2])

    def checkRandomizable(self):
        incomplete = False
        missingKeys = ""
        for key in self._randomize_keys:
            if key not in self._setup.keys():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise SubGoalIncompleteError(
                "Missing keys for randomize: %s" % missingKeys[:-2]
            )
        if self.m() != len(self._setup["low"]):
            raise SubGoalMissmatchDimensionError("Dimension mismatch between low and m")
        if self.m() != len(self._setup["high"]):
            raise SubGoalMissmatchDimensionError(
                "Dimension mismatch between high and m"
            )

    def name(self):
        return self._name

    def dynamic(self):
        return self._dynamic

    def isPrimeGoal(self):
        return self._setup["prime"]

    def indices(self):
        return self._setup["indices"]

    def type(self):
        return self._setup['type']

    def m(self):
        return self._setup["m"]

    def parentLink(self):
        return self._setup["parent_link"]

    def childLink(self):
        return self._setup["child_link"]

    def w(self):
        return self._setup["w"]

    def isJointSpaceGoal(self):
        if 'jointSpace' in self._setup and self._setup['jointSpace']:
            return True
        else:
            return False

    def desiredPosition(self):
        if 'ctrlpts' in self._setup['desired_position']:
            return self._setup['desired_position']['ctrlpts'][-1]
        desPos = []
        if 'trajFun' in self._setup['desired_position']:
            return self._setup['desired_position']['trajFun']
        for x in self._setup["desired_position"]:
            if isinstance(x, float):
                desPos.append(float(x))
            elif isinstance(x, str):
                desPos.append(eval(x))
        return desPos

    def trajectory(self):
        return self._desiredPosition

    def initializeStaticGoal(self):
        pos = [float(x) for x in self._setup['desired_position']]
        self._desiredPosition = lambda t : [np.array(pos), np.zeros(self.m()), np.zeros(self.m())]

    def initializeSplineTrajectory(self):
        degree = self._setup['desired_position']['degree']
        ctrlpts = self._setup['desired_position']['ctrlpts']
        duration = self._setup['desired_position']['duration']
        self._desiredPosition = SplineTrajectory(
            self.m(),
            ca.SX(np.identity(self.m())),
            degree=degree,
            ctrlpts=ctrlpts,
            duration=duration,
        )
        self._desiredPosition.concretize()

    def initializeAnalyticTrajectory(self):
        trajFun = [eval(x) for x in self._setup['desired_position']['trajFun']]
        self._desiredPosition = AnalyticTrajectory(
            self.m(), ca.SX(np.identity(self.m())), traj=trajFun, t=t, name="goal"
        )
        self._desiredPosition.concretize()

    def evaluate(self, t):
        if self.dynamic():
            return self._desiredPosition.evaluate(t)
        else:
            return self._desiredPosition(t)

    def generateRandomPosition(self):
        self.checkRandomizable()
        new_desired_position = np.zeros(self.m())
        for i in range(self.m()):
            new_desired_position[i] = np.random.uniform(
                low=float(self._setup["low"][i]), high=float(self._setup["high"][i])
            )
        self._setup["desired_position"] = [
            str(x) for x in new_desired_position.tolist()
        ]
        self.initializeStaticGoal()

    def goalDict(self):
        return self._setup


if __name__ == "__main__":
    goalDict = {
        "m": 2,
        "w": 1,
        "prime": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 1,
        "desired_position": ["1 * 3 * t", "1.0 * ca.cos(0.5 * t)"],
        "low": ["-1.0", "-1.0"],
        "high": ["1.0", "1.0"],
        "dynamic": True,
    }
    subGoal = SubGoal("simpleSubGoal", goalDict)
    # subGoal.generateRandomPosition()
    print(subGoal.desiredPosition())
    print(subGoal.evaluate(0.1))
    # print(subGoal.goalDict())
