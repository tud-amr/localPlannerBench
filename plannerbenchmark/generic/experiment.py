import yaml
import csv
import gym
import numpy as np
import casadi as ca
from omegaconf import OmegaConf, MISSING
from dataclasses import dataclass
from typing import List, Dict, Optional, Any

import planarenvs.point_robot
import planarenvs.n_link_reacher
import planarenvs.ground_robots
import urdfenvs.tiago_reacher
import urdfenvs.panda_reacher
import urdfenvs.mobile_reacher
import urdfenvs.albert_reacher
import urdfenvs.boxer_robot

from forwardkinematics.fksCommon.fk_creator import FkCreator
from MotionPlanningEnv.obstacleCreator import ObstacleCreator
from MotionPlanningGoal.goalComposition import GoalComposition, SubGoalConfig 
from MotionPlanningEnv.sphereObstacle import SphereObstacleConfig 


class ExperimentIncompleteError(Exception):
    pass


class InvalidInitStateError(Exception):
    pass


class ExperimentInfeasible(Exception):
    pass

@dataclass
class SelfCollisionConfig:
    """Class comment to be filled in"""

    pairs: Optional[List[List[int]]] = MISSING

@dataclass
class StateConfig:
    """Class comment to be filled in"""

    q0: List[float] = MISSING
    q0dot: List[float] = MISSING

@dataclass
class ExperimentConfig:
    """Data class that specifies the structure and default values of an experiment config"""

    T: int = 2000 
    n: int = MISSING
    dynamic: bool = MISSING
    dt: float = 0.05 
    env: str = MISSING
    robot_type: str = MISSING
    # NOTE: Using type Any here, because it's not possible to have a union of custom dataclasses. 
    # Also there is already type checking in the motion-planning-scenes package when instatiating the goals.
    goal: Dict[str, Any] = MISSING
    initState: StateConfig = MISSING
    limits: Dict[str, List[float]] = MISSING
    r_body: float = MISSING
    randomObstacles: Dict[str, int] = MISSING
    obstacles: Optional[Dict[str, SphereObstacleConfig]] = MISSING
    selfCollision: SelfCollisionConfig = MISSING

class Experiment(object):
    def __init__(self, cfg):
        if isinstance(cfg, str):
            cfg = OmegaConf.load()

        self._required_keys = [
            "T",
            "dt",
            "env",
            "n",
            "goal",
            "initState",
            "robot_type",
            "limits",
            "obstacles",
            "r_body",
            "selfCollision",
            "dynamic",
        ]
        self._cfg = cfg
        self._fk = FkCreator(self.robotType(), self.n()).fk()
        self._motionPlanningGoal = GoalComposition(name="mpg", contentDict=self._cfg.goal)
        self.parseObstacles()

    def parseObstacles(self):
        self._obstacles = []
        self._obstacleCreator = ObstacleCreator()
        if self._cfg.obstacles:
            for obst in self._cfg["obstacles"]:
                obstData = self._cfg["obstacles"][obst]
                obstType = obstData['type']
                obstName = obst
                self._obstacles.append(self._obstacleCreator.createObstacle(obstType, obstName, obstData))

    def dynamic(self):
        return self._cfg['dynamic']

    def selfCollisionPairs(self):
        if self._cfg['selfCollision']['pairs']:
            return self._cfg["selfCollision"]["pairs"]
        else:
            return []

    def rBody(self):
        return self._cfg["r_body"]

    def fk(self, q, n, positionOnly=False):
        return self._fk.fk(q, n, positionOnly=positionOnly)

    def evaluate(self, t):
        evalObsts = self.evaluateObstacles(t=t)
        evalGoal = self._motionPlanningGoal.evaluate(t=t)
        return evalGoal + evalObsts

    def evaluateObstacles(self, t):
        evals = []
        for obst in self._obstacles:
            if 'analytic' in obst.type():
                evals += obst.traj().evaluate(t=t)
        return evals

    def robotType(self):
        return self._cfg["robot_type"]

    def n(self):
        return self._cfg["n"]

    def T(self):
        return self._cfg["T"]

    def dt(self):
        return self._cfg["dt"]

    def envName(self):
        return self._cfg["env"]

    def obstacles(self):
        return self._obstacles

    def limits(self):
        low = np.array(self._cfg["limits"]["low"])
        high = np.array(self._cfg["limits"]["high"])
        return low, high

    def initState(self):
        try:
            q0 = np.array([float(x) for x in self._cfg["initState"]["q0"]])
            q0dot = np.array([float(x) for x in self._cfg["initState"]["q0dot"]])
        except:
            raise InvalidInitStateError("Initial state could not be parsed")
        """
        if q0.size != self.n() or q0dot.size != self.n():
            raise InvalidInitStateError("Initial state of wrong dimension")
        """
        return (q0, q0dot)

    def goal(self):
        return self._motionPlanningGoal

    def primeGoal(self, **kwargs):
        return self._motionPlanningGoal.primeGoal()
        if 't' in kwargs:
            return self._motionPlanningGoal.evaluatePrimeGoal(kwargs.get('t'))
        else:
            return self._motionPlanningGoal.primeGoal()

    def evaluatePrimeGoal(self, t):
        return self.primeGoal().position(t=t)

    def getDynamicGoals(self):
        return self._motionPlanningGoal.dynamicGoals()

    def env(self, render=False):
        if self.robotType() == 'planarArm':
            return gym.make(self.envName(), render=render, n=self.n(), dt=self.dt())
        else:
            return gym.make(self.envName(), render=render, dt=self.dt())

    def addScene(self, env):
        for obst in self._obstacles:
            env.add_obstacle(obst)
        try:
            env.add_goal(self.goal())
        except Exception as e:
            print(e)

    def shuffleInitConfiguration(self):
        q0_new = np.random.uniform(low=self.limits()[0], high=self.limits()[1])
        self._cfg["initState"]["q0"] = q0_new.tolist()

    def shuffleObstacles(self):
        self._obstacles = []
        obstData = self._cfg["obstacles"]['obst0']
        obstType = obstData['type']
        for i in range(self._cfg["randomObstacles"]["number"]):
            obstName = 'obst' + str(i)
            randomObst = self._obstacleCreator.createObstacle(obstType, obstName, obstData)
            randomObst.shuffle()
            self._obstacles.append(randomObst)

    def shuffle(self, random_obst, random_init, random_goal):
        if random_goal:
            self._motionPlanningGoal.shuffle()
        if random_obst:
            self.shuffleObstacles()
        if random_init:
            self.shuffleInitConfiguration()
        return

    def checkFeasibility(self, checkGoalReachible):
        for o in self.obstacles():
            for i in range(1, self.n() + 1):
                fk = self.fk(self.initState()[0], i, positionOnly=True)
                if self.robotType() == 'boxer':
                    fk = fk[0:2]
                dist_initState = np.linalg.norm(np.array(o.position()) - fk)
                if dist_initState < (o.radius() + self.rBody()):
                    raise ExperimentInfeasible("Initial configuration in collision")
            if not self.dynamic() and len(o.position()) == len(self.primeGoal().position()):
                dist_goal = np.linalg.norm(np.array(o.position()) - self.primeGoal().position())
                if dist_goal < (o.radius() + self.rBody()):
                    raise ExperimentInfeasible("Goal in collision")
        for pair in self.selfCollisionPairs():
            fk1 = self.fk(self.initState()[0], pair[0], positionOnly=True)
            fk2 = self.fk(self.initState()[0], pair[1], positionOnly=True)
            dist_initState = np.linalg.norm(fk1 - fk2)
            if dist_initState < (2 * self.rBody()):
                raise ExperimentInfeasible(
                    "Initial configuration in self collision"
                )
        if self.robotType() == "planarArm":
            if np.linalg.norm(np.array(self.primeGoal().position())) > self.n():
                raise ExperimentInfeasible("Goal unreachible")

    def save(self, folderPath):
        self._cfg["goal"] = self._motionPlanningGoal.toDict()
        obstsDict = {}
        obstFile = folderPath + "/obst"
        initStateFilename = folderPath + "/initState.csv"
        for i, obst in enumerate(self._obstacles):
            obstsDict[obst.name()] = obst.toDict()
            obst.toCSV(obstFile + "_" + str(i) + ".csv")
        self._cfg["obstacles"] = obstsDict
        with open(folderPath + "/exp.yaml", "w") as file:
            OmegaConf.save(config=self._cfg, f=file)
        with open(initStateFilename, "w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            csv_writer.writerow(self.initState()[0])

