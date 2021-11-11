import numpy as np
import casadi as ca
import csv

from fabricsExperiments.infrastructure.subGoal import SubGoal
from fabricsExperiments.infrastructure.variables import t


class NoSubgoalsError(Exception):
    pass


class MotionPlanningGoalRandomizeError(Exception):
    pass


class MotionPlanningGoal(object):
    def __init__(self, goalsDict):
        self._dynamic = False
        self._goalsDict = goalsDict
        self._subGoals = []
        self.parseGoalsDict(goalsDict)

    def dynamic(self):
        return self._dynamic

    def parseGoalsDict(self, goalsDict):
        if not goalsDict.keys():
            raise NoSubgoalsError("No subgoals defined for the motion planning Goal")
        for subGoal in goalsDict.keys():
            sg = SubGoal(subGoal, goalsDict[subGoal])
            if sg.dynamic():
                self._dynamic = True
            self._subGoals.append(sg)

    def shuffle(self):
        for subGoal in self._subGoals:
            try:
                if not subGoal.dynamic():
                    subGoal.generateRandomPosition()
            except Exception as e:
                raise MotionPlanningGoalRandomizeError(
                    "Randomize error for %s with error: %s" % (subGoal.name(), str(e))
                )

    def subGoalsDict(self):
        goalsDict = {}
        for subGoal in self._subGoals:
            goalsDict[subGoal.name()] = subGoal.goalDict()
        return goalsDict

    def subGoals(self):
        return self._subGoals

    def primeGoal(self):
        for subGoal in self._subGoals:
            if subGoal.isPrimeGoal():
                return subGoal.desiredPosition()

    def evaluatePrimeGoal(self, t):
        for subGoal in self._subGoals:
            if subGoal.isPrimeGoal():
                return subGoal.evaluate(t)

    def evaluate(self, t):
        evaluations = []
        for subGoal in self._subGoals:
            if subGoal.dynamic():
                evaluations += subGoal.evaluate(t)
        return evaluations

    def toCSV(self, folderPath):
        goalFilename = folderPath + "/goal.csv"
        with open(goalFilename, "w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            for subGoal in self.subGoals():
                if subGoal.parentLink() == 0:
                    csv_writer.writerow(subGoal.desiredPosition())


if __name__ == "__main__":
    goalDict1 = {
        "m": 2,
        "w": 1,
        "prime": True,
        "indices": [0, 1],
        "parent_link": 0,
        "child_link": 1,
        "desired_position": ["0.0", "1.0"],
        "low": ["-1.0", "-1.0"],
        "high": ["1.0", "1.0"],
        "dynamic": False
    }
    goalDict2 = {
        "m": 1,
        "w": 1,
        "prime": False,
        "indices": [0],
        "parent_link": 0,
        "child_link": 1,
        "desired_position": ["1 * t"],
        "low": ["-1.0"],
        "high": ["1.0"],
        "dynamic": True
    }
    goal = {"subgoal1": goalDict1, "subgoal2": goalDict2}
    motionPlanningGoal = MotionPlanningGoal(goal)
    motionPlanningGoal.shuffle()
    print(motionPlanningGoal.subGoalsDict())
    print(motionPlanningGoal.evaluate(0.1))
