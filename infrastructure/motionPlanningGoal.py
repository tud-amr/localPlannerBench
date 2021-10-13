import numpy as np

class MotionPlanningGoal(object):
    def __init__(self, goalsDict):
        self._goals = []
        self.parseGoalsDict(goalsDict)

    def parseGoalsDict(self, goalsDict):
        for goal in goalsDict.keys():
            self._goals.append(PositionGoal(goal, goalsDict[goal]))

    def generateRandomPosition(self):
        for goal in self._goals:
            goal.generateRandomPosition()

    def toDict(self):
        goalsDict = {}
        for goal in self._goals:
            goalsDict[goal._name] = goal.toDict()


class PositionGoal(object):
    def __init__(self, name, goalDict):
        self._name = name
        self.parseGoalDict(goalDict)

    def parseGoalDict(self, goalDict):
        self._m = goalDict['m']
        self._indices = goalDict['indices']
        self._parent_link = goalDict['parent_link']
        self._child_link = goalDict['child_link']
        self._desired_position = np.array(goalDict['desired_position'])
        self._low = goalDict['low']
        self._high = goalDict['high']
        self._prime = goalDict['prime']
        self._w = goalDict['w']

    def isPrimeGoal(self):
        return self._prime

    def getIndices(self):
        return self._indices

    def generateRandomPosition(self):
        for i in range(self._m):
            self._desired_position[i] = np.random.uniform(
                                            low=self._low[i], high=self._high[i]
                                        )

    def toDict(self):
        goalDict = {}
        goalDict['parent_link'] = self._parent_link
        goalDict['child_link'] = self._child_link
        goalDict['desired_position'] = self._desired_position.tolist()
        goalDict['m'] = self._m
        goalDict['low'] = self._low
        goalDict['high'] = self._high
        return goalDict
