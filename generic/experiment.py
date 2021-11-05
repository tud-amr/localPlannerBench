import yaml
import csv
import gym
import numpy as np
import casadi as ca

import pointRobot
import pandaReacher
import nLinkReacher

from casadiFk import ForwardKinematics
from obstacle import Obstacle

from fabricsExperiments.infrastructure.motionPlanningGoal import MotionPlanningGoal


class ExperimentIncompleteError(Exception):
    pass


class InvalidInitStateError(Exception):
    pass


class ExperimentInfeasible(Exception):
    pass


class Experiment(object):
    def __init__(self, setupFile):
        self._setupFile = setupFile
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
        ]
        self.parseSetup()
        self._fk = ForwardKinematics(robot_type=self.robotType())

    def parseSetup(self):
        with open(self._setupFile, "r") as setupStream:
            self._setup = yaml.safe_load(setupStream)
        self.checkCompleteness()
        self._motionPlanningGoal = MotionPlanningGoal((self._setup["goal"]))
        self._dynamic = self._motionPlanningGoal.dynamic()
        self._obstacles = []
        if self._setup["obstacles"]:
            for obst in self._setup["obstacles"]:
                obstData = self._setup["obstacles"][obst]
                self._obstacles.append(
                    Obstacle([float(xi) for xi in obstData["x"]], obstData["r"])
                )

    def dynamic(self):
        return self._dynamic

    def selfCollisionPairs(self):
        return self._setup["selfCollision"]["pairs"]

    def rBody(self):
        return self._setup["r_body"]

    def fk(self, q, n, positionOnly=False):
        return self._fk.getFk(q, n, positionOnly=positionOnly)

    def evaluate(self, t):
        return self._motionPlanningGoal.evaluate(t)

    def robotType(self):
        return self._setup["robot_type"]

    def n(self):
        return self._setup["n"]

    def T(self):
        return self._setup["T"]

    def dt(self):
        return self._setup["dt"]

    def envName(self):
        return self._setup["env"]

    def obstacles(self):
        return self._obstacles

    def limits(self):
        low = np.array(self._setup["limits"]["low"])
        high = np.array(self._setup["limits"]["high"])
        return low, high

    def initState(self):
        try:
            q0 = np.array([float(x) for x in self._setup["initState"]["q0"]])
            q0dot = np.array([float(x) for x in self._setup["initState"]["q0dot"]])
        except:
            raise InvalidInitStateError("Initial state could not be parsed")
        if q0.size != self.n() or q0dot.size != self.n():
            raise InvalidInitStateError("Initial state of wrong dimension")
        return (q0, q0dot)

    def goal(self):
        return self._motionPlanningGoal

    def primeGoal(self, **kwargs):
        if 't' in kwargs:
            return self._motionPlanningGoal.evaluatePrimeGoal(kwargs.get('t'))
        else:
            return self._motionPlanningGoal.primeGoal()

    def evaluatePrimeGoal(self, t):
        return self._motionPlanningGoal.evaluatePrimeGoal(t)

    def getDynamicGoals(self):
        return self._motionPlanningGoal.dynamicGoals()

    def env(self, render=False):
        return gym.make(self.envName(), render=render, n=self.n(), dt=self.dt())

    def addScene(self, env):
        for obst in self._obstacles:
            env.addObstacle(pos=obst.x(), filename='sphere05red_nocol.urdf')
        env.addObstacle(pos=self.primeGoal(), filename="sphere_goal.urdf")

    def shuffleInitConfiguration(self):
        q0_new = np.random.uniform(low=self.limits()[0], high=self.limits()[1])
        self._setup["initState"]["q0"] = q0_new.tolist()

    def shuffleObstacles(self):
        self._obstacles = []
        for i in range(self._setup["randomObstacles"]["number"]):
            pos = np.random.uniform(
                low=self._setup["randomObstacles"]["limits"]["x"]["low"],
                high=self._setup["randomObstacles"]["limits"]["x"]["high"],
            )
            r = float(
                np.random.uniform(
                    low=self._setup["randomObstacles"]["limits"]["r"]["low"],
                    high=self._setup["randomObstacles"]["limits"]["r"]["high"],
                )
            )
            self._obstacles.append(Obstacle(pos, r))

    def shuffle(self, random_obst, random_init, random_goal):
        if random_goal:
            self._motionPlanningGoal.shuffle()
        if random_obst:
            self.shuffleObstacles()
        if random_init:
            self.shuffleInitConfiguration()
        return

    def checkCompleteness(self):
        incomplete = False
        missingKeys = ""
        for key in self._required_keys:
            if key not in self._setup.keys():
                incomplete = True
                missingKeys += key + ", "
        if incomplete:
            raise ExperimentIncompleteError("Missing keys: %s" % missingKeys[:-2])

    def checkFeasibility(self, checkGoalReachible):
        for o in self.obstacles():
            for i in range(0, self.n() + 1):
                fk = self.fk(self.initState()[0], i, positionOnly=True)
                dist_initState = np.linalg.norm(np.array(o.x()) - fk)
                if dist_initState < (o.r() + self.rBody()):
                    raise ExperimentInfeasible("Initial configuration in collision")
            if len(self.primeGoal()) == len(o.x()) and isinstance(self.primeGoal(), np.ndarray):
                dist_goal = np.linalg.norm(np.array(o.x()) - self.primeGoal())
                if dist_goal < (o.r() + self.rBody()):
                    raise ExperimentInfeasible("Goal in collision")
        if self.robotType() != "pointMass":
            for i in range(self.n() + 1):
                fk1 = self.fk(self.initState()[0], i, positionOnly=True)
                for j in range(i + 2, self.n() + 1):
                    fk2 = self.fk(self.initState()[0], j, positionOnly=True)
                    dist_initState = np.linalg.norm(fk1 - fk2)
                    if dist_initState < (2 * self.rBody()):
                        raise ExperimentInfeasible(
                            "Initial configuration in self collision"
                        )
            if np.linalg.norm(self.primeGoal()) > self.n():
                raise Experiment("Goal unreachible")

    def save(self, folderPath):
        self._setup["goal"] = self._motionPlanningGoal.subGoalsDict()
        obstsDict = {}
        obstFile = folderPath + "/obst"
        initStateFilename = folderPath + "/initState.csv"
        for i, obst in enumerate(self._obstacles):
            obstsDict["obst" + str(i)] = obst.toDict()
            obst.toCSV(obstFile + "_" + str(i) + ".csv")
        self._setup["obstacles"] = obstsDict
        with open(folderPath + "/exp.yaml", "w") as file:
            yaml.dump(self._setup, file)
        self._motionPlanningGoal.toCSV(folderPath)
        with open(initStateFilename, "w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            csv_writer.writerow(self.initState()[0])

