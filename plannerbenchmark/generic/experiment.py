import yaml
import csv
import gym
import numpy as np
import logging
from copy import deepcopy

import planarenvs.point_robot
import planarenvs.n_link_reacher
import planarenvs.ground_robots
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.robots.albert import AlbertRobot
from urdfenvs.robots.tiago import TiagoRobot
from urdfenvs.robots.boxer import BoxerRobot

from forwardkinematics.fksCommon.fk_creator import FkCreator
from MotionPlanningEnv.obstacleCreator import ObstacleCreator
from MotionPlanningGoal.staticSubGoal import StaticSubGoal
from MotionPlanningGoal.goalComposition import GoalComposition

def create_robot(robot_type, control_mode):
    if robot_type == 'albert':
        robot = AlbertRobot(mode=control_mode)
    elif robot_type == 'tiago':
        robot = TiagoRobot(mode=control_mode)
    elif robot_type == 'boxer':
        robot = BoxerRobot(mode=control_mode)
    else:
        robot = GenericUrdfReacher(urdf=robot_type+".urdf", mode=control_mode)
    return robot



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
            "n",
            "control_mode",
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
        self._fk = FkCreator(self.robot_type(), self.n()).fk()

    def parseSetup(self):
        with open(self._setupFile, "r") as setupStream:
            self._setup = yaml.safe_load(setupStream)
        self.checkCompleteness()
        self._motionPlanningGoal = GoalComposition(name="mpg", content_dict=self._setup['goal'])
        self._original_setup = deepcopy(self._setup)
        self.parseObstacles()

    def parseObstacles(self):
        self._obstacles = []
        self._obstacleCreator = ObstacleCreator()
        if self._setup["obstacles"]:
            for obst in self._setup["obstacles"]:
                obstData = self._setup["obstacles"][obst]
                obstType = obstData['type']
                obstName = obst
                self._obstacles.append(self._obstacleCreator.create_obstacle(obstType, obstName, obstData))

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
        dimension_goal = evalGoal[0].size
        for i in range(3):
            if np.isnan(evalGoal[i]).any():
                evalGoal[i] = np.zeros(dimension_goal)

        return {'goal': evalGoal, 'obstacles': evalObsts}

    def evaluateObstacles(self, t):
        evals = []
        for obst in self._obstacles:
            evals.append([obst.position(t=t), obst.velocity(t=t), obst.acceleration(t=t), obst.radius()])
        return evals

    def robot_type(self):
        return self._setup["robot_type"]

    def n(self):
        return self._setup["n"]

    def T(self):
        return self._setup["T"]

    def dt(self):
        return self._setup["dt"]

    def control_mode(self):
        return self._setup["control_mode"]


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

    def primary_goal(self, **kwargs):
        return self._motionPlanningGoal.primary_goal(**kwargs)

    def evaluate_primary_goal(self, t):
        return self.primary_goal().evaluate(t=t)

    def getDynamicGoals(self):
        return self._motionPlanningGoal.dynamicGoals()

    def env(self, render=False):
        if self.robot_type() in ["planarArm", "pointRobot"]:
            return gym.make(self.env_name(), render=render, n=self.n(), dt=self.dt())
        else:
            robots = [create_robot(self.robot_type(), self.control_mode())]
            return gym.make("urdf-env-v0", robots=robots, render=render, dt=self.dt())

    def env_name(self):
        return self._setup['env']

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
            randomObst = self._obstacleCreator.create_obstacle(obstType, obstName, obstData)
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

    def restore_original_goal(self):
        self._motionPlanningGoal = GoalComposition(name = "mpg", content_dict = self._original_setup['goal'])
 
    def compute_global_path(self):
        logging.info("Creating global plan using OMPL.")
        from plannerbenchmark.generic.global_planner import GlobalPlanner
        global_planner = GlobalPlanner(self.obstacles(), self.primary_goal().dimension())
        start = self.fk(self.initState()[0], self.n(), positionOnly=True)
        if not isinstance(start, np.ndarray):
            start = np.array([float(start[i]) for i in range(len(start))])
        goal = self.primary_goal().position()
        global_planner.initialize_search_space(low=[-0.0, -1.0, 0.2], high=[1.0, 1.0, 1.2])
        global_planner.setup_planning_problem(start, goal)
        global_planner.solve_planning_problem()

        self._setup['goal']['subgoal0']['type'] = 'splineSubGoal'
        self._setup['goal']
        new_setup = self._motionPlanningGoal.dict()
        new_setup['subgoal0']['type'] = 'splineSubGoal'
        self._setup['dynamic'] = True
        new_setup['subgoal0'].pop('desired_position', None)
        new_setup['subgoal0']['trajectory'] = {
                'controlPoints': global_planner.control_points(),
                'degree': 2,
                'duration': 10,
            }
        self._motionPlanningGoal = GoalComposition(name="mpg", content_dict=new_setup)

    def checkFeasibility(self, checkGoalReachible):
        for o in self.obstacles():
            for i in range(1, self.n() + 1):
                fk = self.fk(self.initState()[0], i, positionOnly=True)
                if self.robot_type() == 'boxer':
                    fk = fk[0:2]
                dist_initState = np.linalg.norm(np.array(o.position()) - fk)
                if dist_initState < (o.radius() + self.rBody()):
                    raise ExperimentInfeasible("Initial configuration in collision")
            if not self.dynamic() and len(o.position()) == len(self.primary_goal().position()):
                dist_goal = np.linalg.norm(np.array(o.position()) - self.primary_goal().position())
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
        if self.robot_type() == "planarArm":
            if np.linalg.norm(np.array(self.primary_goal().position())) > self.n():
                raise ExperimentInfeasible("Goal unreachible")

    def save(self, folderPath):
        self._setup["goal"] = self._motionPlanningGoal.dict()
        obstsDict = {}
        obstFile = folderPath + "/obst"
        initStateFilename = folderPath + "/initState.csv"
        for i, obst in enumerate(self._obstacles):
            obstsDict[obst.name()] = obst.dict()
            obst.csv(obstFile + "_" + str(i) + ".csv")
        self._setup["obstacles"] = obstsDict
        with open(folderPath + "/exp.yaml", "w") as file:
            yaml.dump(self._setup, file, default_flow_style=False)
        with open(initStateFilename, "w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            csv_writer.writerow(self.initState()[0])

