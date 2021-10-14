import numpy as np
import csv

from fabricsExperiments.infrastructure.subGoal import SubGoal


class NoSubgoalsError(Exception):
    pass


class MotionPlanningGoalRandomizeError(Exception):
    pass


class MotionPlanningGoal(object):
    def __init__(self, goalsDict):
        self._goalsDict = goalsDict
        self._subGoals = []
        self.parseGoalsDict(goalsDict)

    def parseGoalsDict(self, goalsDict):
        if not goalsDict.keys():
            raise NoSubgoalsError("No subgoals defined for the motion planning Goal")
        for subGoal in goalsDict.keys():
            self._subGoals.append(SubGoal(subGoal, goalsDict[subGoal]))

    def shuffle(self):
        for subGoal in self._subGoals:
            try:
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
    }
    goalDict2 = {
        "m": 1,
        "w": 1,
        "prime": False,
        "indices": [0],
        "parent_link": 0,
        "child_link": 1,
        "desired_position": ["0.0"],
        "low": ["-1.0"],
        "high": ["1.0"],
    }
    goal = {"subgoal1": goalDict1, "subgoal2": goalDict2}
    motionPlanningGoal = MotionPlanningGoal(goal)
    motionPlanningGoal.shuffle()
    print(motionPlanningGoal.subGoalsDict())
