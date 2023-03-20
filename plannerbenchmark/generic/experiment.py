from mpscenes.goals.sub_goal_creator import StaticSubGoal
import yaml
import os
import csv
import gym
import numpy as np
import logging

from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.robots.generic_urdf import GenericDiffDriveRobot
from urdfenvs.sensors.full_sensor import FullSensor

from forwardkinematics.urdfFks.generic_urdf_fk import GenericURDFFk
from mpscenes.goals.goal_composition import GoalComposition
from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle


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
            "goal",
            "initState",
            "base_type",
            "limits",
            "control_mode",
            "urdf_file_name",
            "root_link",
            "ee_links",
            "obstacles",
            "r_body",
            "selfCollision",
        ]
        self.parseSetup()
        self._fk = GenericURDFFk(
            self.urdf(),
            rootLink=self._setup['root_link'],
            end_link = self._setup['ee_links'],
            base_type=self.base_type(),
        )

    def urdf(self):
        with open(self.urdf_file(), 'r') as file:
            urdf = file.read()
        return urdf
    
    def urdf_file(self):

        abs_path_experiment = os.path.abspath(__file__)
        asset_folder = abs_path_experiment[0:abs_path_experiment.rfind('k/')+2] + 'assets/'
        return asset_folder+self._setup['urdf_file_name']


    def parseSetup(self):
        with open(self._setupFile, "r") as setupStream:
            self._setup = yaml.safe_load(setupStream)
        self.checkCompleteness()
        self._motionPlanningGoal = GoalComposition(name="mpg", content_dict=self._setup['goal'])
        self.parseObstacles()

    def parseObstacles(self):
        self._obstacles = []
        if self._setup["obstacles"]:
            for obst in self._setup["obstacles"]:
                obstData = self._setup["obstacles"][obst]
                if 'position' in obstData['geometry']:
                    obstacle = SphereObstacle(name=obst, content_dict=obstData)
                elif 'trajectory' in obstData['geometry']:
                    obstacle = DynamicSphereObstacle(name=obst, content_dict=obstData)
                self._obstacles.append(obstacle)

    def dynamic(self):
        return True

    def selfCollisionPairs(self):
        if self._setup['selfCollision']['pairs']:
            return self._setup["selfCollision"]["pairs"]
        else:
            return []

    def collision_links(self) -> list:
        if 'collision_links' in self._setup:
            return self._setup['collision_links']
        else:
            logging.warning("No collision links provided.")
            return []

    def rBody(self):
        return self._setup["r_body"]

    def fk(self, q, n, positionOnly=False):
        return self._fk.fk(q, self._setup['root_link'], n, positionOnly=positionOnly)

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

    def base_type(self):
        return self._setup["base_type"]

    def root_link(self) -> str:
        return self._setup['root_link']

    def ee_links(self) -> list:
        return self._setup['ee_links']

    def n(self):
        return self._setup["n"]

    def T(self):
        return self._setup["T"]

    def dt(self):
        return self._setup["dt"]

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

    def control_mode(self) -> str:
        return self._setup['control_mode']

    def primeGoal(self, **kwargs) -> StaticSubGoal: 
        return self._motionPlanningGoal.primary_goal()

    def evaluatePrimeGoal(self, t) -> np.ndarray:
        return self.primeGoal().position(t=t)

    def getDynamicGoals(self):
        return self._motionPlanningGoal.dynamicGoals()

    def env(self, render=False):
        if self.base_type() == 'holonomic':
            robot = GenericUrdfReacher(
                self.urdf_file(),
                mode = self.control_mode(),
            )
        elif self.base_type() == 'diffdrive':
            robot = GenericDiffDriveRobot(
                urdf=self.urdf_file(),
                mode=self.control_mode(),
                actuated_wheels=self._setup['actuated_wheels'],
                castor_wheels=self._setup['castor_wheels'],
                wheel_radius=self._setup['wheel_radius'],
                wheel_distance=self._setup['wheel_distance'],
                not_actuated_joints=self._setup['not_actuated_joints'],
            )

        env = gym.make(
            "urdf-env-v0",
            dt=self.dt(), robots=[robot], render=render
        )
        self.addScene(env)
        return env


    def addScene(self, env):
        for obst in self._obstacles:
            env.add_obstacle(obst)
        try:
            for sub_goal in self.goal().sub_goals():
                env.add_goal(sub_goal)
        except Exception as e:
            logging.error(f"Error occured when adding goal to the scene, {e}")
        full_sensor = FullSensor(
            goal_mask=["position"],
            obstacle_mask=["position", "velocity", "size"],
            variance=0,
        )
        env.add_sensor(full_sensor, robot_ids = [0])
        env.set_spaces()

    def shuffleInitConfiguration(self):
        q0_new = np.random.uniform(low=self.limits()[0], high=self.limits()[1])
        self._setup["initState"]["q0"] = q0_new.tolist()

    def shuffleObstacles(self):
        self._obstacles = []
        obstData = self._setup["obstacles"]['obst0']
        obstType = obstData['type']
        for i in range(self._setup["randomObstacles"]["number"]):
            obstName = 'obst' + str(i)
            if 'position' in obstData['geometry']:
                obstacle = SphereObstacle(name=obstName, content_dict=obstData)
            elif 'trajectory' in obstData['geometry']:
                obstacle = DynamicSphereObstacle(name=obstName, content_dict=obstData)
            obstacle.shuffle()
            self._obstacles.append(obstacle)

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
            for collision_link in self.collision_links():
                fk = self.fk(self.initState()[0], collision_link, positionOnly=True)
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
            yaml.dump(self._setup, file)
        with open(initStateFilename, "w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            csv_writer.writerow(self.initState()[0])

