import numpy as np


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
        ]
        self._randomize_keys = ["low", "high"]
        self._setup = goalDict
        self.checkCompleteness()
        self.checkDimensionality()

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

    def isPrimeGoal(self):
        return self._setup["prime"]

    def indices(self):
        return self._setup["indices"]

    def m(self):
        return self._setup["m"]

    def parentLink(self):
        return self._setup["parent_link"]

    def childLink(self):
        return self._setup["child_link"]

    def w(self):
        return self._setup["w"]

    def desiredPosition(self):
        return [float(x) for x in self._setup["desired_position"]]

    def generateRandomPosition(self):
        self.checkRandomizable()
        new_desired_position = np.zeros(self.m())
        for i in range(self.m()):
            new_desired_position[i] = np.random.uniform(
                low=float(self._setup["low"][i]), high=float(self._setup["high"][i])
            )
        self._setup["desired_position"] = new_desired_position.tolist()

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
        "desired_position": ["0.0", "1.0"],
        "low": ["-1.0", "-1.0"],
        "high": ["1.0", "1.0"],
    }
    subGoal = SubGoal("simpleSubGoal", goalDict)
    subGoal.generateRandomPosition()
    print(subGoal.desiredPosition())
    print(subGoal.goalDict())
