from dataclasses import dataclass
from mpscenes.goals.goal_composition import GoalComposition
import numpy as np
import logging

from plannerbenchmark.generic.planner import Planner, PlannerConfig

@dataclass
class PdConfig(PlannerConfig):
    k: float = 0.8
    p: float = 0.2


class PDPlanner(Planner):
    def __init__(self, exp, **kwargs):
        super().__init__(exp, **kwargs)
        self._config = PdConfig(**kwargs)
        self._curError = np.zeros(self.config.n)
        logging.warn("The PD-Planner that you are using is not able to deal with obstacles")

    def dt(self):
        return self._exp.dt()

    def reset(self):
        pass

    def setObstacles(self, obsts, r_body):
        pass

    def setSelfCollisionAvoidance(self, r_body):
        pass

    def setJointLimits(self, limits):
        pass

    def setGoal(self, goal: GoalComposition):
        self._goal_position = goal.primary_goal().position()
        pass

    def concretize(self):
        pass

    def evalError(self, q):
        newError = self._goal_position - q
        self._curErrorDot = (newError - self._curError) / self.dt()
        self._curError = newError

    def computeAction(self, **kwargs):
        self.evalError(kwargs['joint_state']['position'][0:2])
        action = self.config.p * self._curError + self.config.k * self._curErrorDot
        action = np.concatenate((action, np.zeros(1)))
        return action

