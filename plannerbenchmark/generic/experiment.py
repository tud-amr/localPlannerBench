import yaml
import csv
import gym
import numpy as np
import logging

import planarenvs.point_robot
import planarenvs.n_link_reacher
import planarenvs.ground_robots
import urdfenvs.tiago_reacher
import urdfenvs.panda_reacher
import urdfenvs.mobile_reacher
import urdfenvs.albert_reacher
import urdfenvs.boxer_robot
import urdfenvs.point_robot_urdf

from forwardkinematics.fksCommon.fk_creator import FkCreator
from MotionPlanningEnv.obstacleCreator import ObstacleCreator
from MotionPlanningGoal.staticSubGoal import StaticSubGoal
from MotionPlanningGoal.goalComposition import GoalComposition



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
            "dynamic",
        ]
        self.parseSetup()
        self._fk = FkCreator(self.robotType(), self.n()).fk()

    def parseSetup(self):
        with open(self._setupFile, "r") as setupStream:
            self._setup = yaml.safe_load(setupStream)
        self.checkCompleteness()
        self._motionPlanningGoal = GoalComposition(name="mpg", contentDict=self._setup['goal'])
        self.parseObstacles()

    def parseObstacles(self):
        self._obstacles = []
        self._obstacleCreator = ObstacleCreator()
        if self._setup["obstacles"]:
            for obst in self._setup["obstacles"]:
                obstData = self._setup["obstacles"][obst]
                obstType = obstData['type']
                obstName = obst
                self._obstacles.append(self._obstacleCreator.createObstacle(obstType, obstName, obstData))

    def dynamic(self):
        return self._setup['dynamic']

    def selfCollisionPairs(self):
        if self._setup['selfCollision']['pairs']:
            return self._setup["selfCollision"]["pairs"]
        else:
            return []

    def rBody(self):
        return self._setup["r_body"]

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
            logging.error(f"Error occured when adding goal to the scene, {e}")

    def shuffleInitConfiguration(self):
        q0_new = np.random.uniform(low=self.limits()[0], high=self.limits()[1])
        self._setup["initState"]["q0"] = q0_new.tolist()

    def shuffleObstacles(self):
        self._obstacles = []
        obstData = self._setup["obstacles"]['obst0']
        obstType = obstData['type']
        for i in range(self._setup["randomObstacles"]["number"]):
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
        self._setup["goal"] = self._motionPlanningGoal.toDict()
        obstsDict = {}
        obstFile = folderPath + "/obst"
        initStateFilename = folderPath + "/initState.csv"
        for i, obst in enumerate(self._obstacles):
            obstsDict[obst.name()] = obst.toDict()
            obst.toCSV(obstFile + "_" + str(i) + ".csv")
        self._setup["obstacles"] = obstsDict
        with open(folderPath + "/exp.yaml", "w") as file:
            yaml.dump(self._setup, file)
        with open(initStateFilename, "w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            csv_writer.writerow(self.initState()[0])

