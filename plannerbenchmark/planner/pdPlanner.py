import numpy as np

from plannerbenchmark.generic.abstractPlanner import AbstractPlanner


class PDPlanner(AbstractPlanner):
    def __init__(self, exp, setupFile):
        required_keys = ["type", "n", 'k', 'p']
        super().__init__(exp, setupFile, required_keys)
        self._curError = np.zeros(self.n())
        if not 'interval' in self._setup:
            self._setup['interval'] = 1

    def interval(self):
        return self._setup['interval']

    def n(self):
        return self._setup['n']

    def dt(self):
        return self._exp.dt()

    def k(self):
        return self._setup['k']

    def p(self):
        return self._setup['p']

    def reset(self):
        pass

    def setObstacles(self, obsts, r_body):
        pass

    def setSelfCollisionAvoidance(self, r_body):
        pass

    def setJointLimits(self, limits):
        pass

    def setGoal(self, goal):
        self._goal_position = goal.primeGoal().position()
        pass

    def concretize(self):
        pass

    def evalError(self, q):
        newError = self._goal_position - q
        self._curErrorDot = (newError - self._curError) / self.dt()
        self._curError = newError

    def computeAction(self, *args):
        self.evalError(args[0])
        return self.p() * self._curError + self.k() * self._curErrorDot

