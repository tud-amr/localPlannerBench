from abc import abstractmethod
from dataclasses import dataclass, asdict
import yaml
from plannerbenchmark.generic.planner_registry import PlannerRegistry

class PlannerSettingIncomplete(Exception):
    pass

@dataclass
class PlannerConfig():
    interval: int = 1
    n: int = 2
    name: str = 'Planner'
    robot_type: str = 'pointRobot'

class Planner(metaclass=PlannerRegistry):
    def __init__(self, exp, **kwargs):
        self._exp = exp
        self._config = PlannerConfig()

    @abstractmethod
    def reset(self):
        pass

    @property
    def config(self):
        return self._config

    def plannerType(self):
        return self.config.name

    def checkCompleteness(self):
        incomplete = False
        missingKeys = ""
        for key in self._required_keys:
            if key not in self._setup.keys():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise PlannerSettingIncomplete("Missing keys: %s" % missingKeys[:-2])

    @abstractmethod
    def setGoal(self, motionPlanningGoal):
        pass

    @abstractmethod
    def setJointLimits(self, lower_limits, upper_limits):
        pass

    @abstractmethod
    def setSelfCollisionAvoidance(self, r_body):
        pass

    @abstractmethod
    def setObstacles(self, obstacles, r_body):
        pass

    @abstractmethod
    def concretize(self):
        pass

    @abstractmethod
    def config_as_dict(self):
        pass

    def save(self, folderPath):
        with open(folderPath + "/planner.yaml", 'w') as file:
            yaml.dump(asdict(self.config), file)

    @abstractmethod
    def computeAction(self, q, qdot):
        pass
