import yaml
import csv
import gym
import pointRobot
import nLinkReacher
import pandaReacher
import numpy as np

from obstacle import Obstacle
from numpyFk import numpyFk

class CaseNotFeasibleError(Exception):
    pass



class ExpSetup(object):
    """docstring for Setup"""

    def __init__(self, setupFile, randomGoal=False, randomObst=False, randomInit=False):
        self._randomGoal = randomGoal
        self._randomObst = randomObst
        self._randomInit = randomInit
        self._setupFile = setupFile
        self.processFile()
        self.setObstacles()

    def processFile(self):
        with open(self._setupFile, "r") as stream:
            self._params = yaml.safe_load(stream)
        if self._randomGoal:
            goal = []
            for i in range(self._params["m"]):
                goal.append(
                    np.random.uniform(
                        low=self._params["randomGoal"]["low"][i],
                        high=self._params["randomGoal"]["high"][i],
                    )
                )
            self._params["goal"] = goal
        if self._randomInit:
            q0 = []
            for i in range(self._params["n"]):
                q0.append(
                    np.random.uniform(
                        low=self._params["randomInit"]["low"][i],
                        high=self._params["randomInit"]["high"][i],
                    )
                )
            self._params["q0"] = q0
        if self._randomObst:
            self._params["obstacles"] = []
            for i in range(self._params["randomObstacle"]["nb"]):
                name = "obst_" + str(i)
                pos = []
                for j in range(self._params["m"]):
                    pos.append(
                        np.random.uniform(
                            low=self._params["randomObstacle"]["pos_low"][j],
                            high=self._params["randomObstacle"]["pos_high"][j],
                        )
                    )
                r = float(np.random.uniform(
                    low=self._params["randomObstacle"]["r_low"],
                    high=self._params["randomObstacle"]["r_high"],
                ))
                obst = {name: name, "x": pos, "r": r}
                self._params["obstacles"].append(obst)

    def makeEnv(self):
        return gym.make(self._params["env"], n=self._params["n"], dt=self._params["dt"])

    def connectRos(self, planner):
        from fabricsExperiments.infrastructure.ros_converter_node import ActionConverterNode
        from fabricsExperiments.infrastructure.ros_integrator_node import IntegratorNode
        dt = self._params["dt"]
        rate_int = 1/self._params["dt"]
        return ActionConverterNode(planner, dt, rate_int, self._params["n"]), IntegratorNode()

    def initialState(self):
        return self._params["q0"], self._params["q0dot"]

    def goal(self):
        return self._params["goal"]

    def setObstacles(self):
        self._obsts = []
        for o in self._params["obstacles"]:
            self._obsts.append(Obstacle(o["x"], o["r"]))

    def getObstacles(self):
        return self._obsts

    def n(self):
        return self._params["n"]

    def T(self):
        return self._params["T"]

    def lowerLimits(self):
        return self._params["limits"]["lower"]

    def upperLimits(self):
        return self._params["limits"]["upper"]

    def checkFeasibility(self, checkGoalReachible=True, checkSelfCollision=True):
        r_body = self._params['r_body']
        for o in self._obsts:
            for i in range(1, self._params["n"] + 1):
                fk = numpyFk(self._params["q0"], i)[0:2]
                dist_initState = np.linalg.norm(np.array(o.x()) - fk)
                if dist_initState < (o.r() + r_body):
                    raise CaseNotFeasibleError("Initial configuration in collision")
            dist_goal = np.linalg.norm(np.array(o.x()) - np.array(self._params["goal"]))
            if dist_goal < (o.r() + r_body):
                raise CaseNotFeasibleError("Goal in collision")
        for i in range(self._params["n"] + 1):
            for j in range(i+2, self._params["n"] + 1):
                fk1 = numpyFk(self._params['q0'], i)[0:2]
                fk2 = numpyFk(self._params['q0'], j)[0:2]
                dist_initState = np.linalg.norm(fk1 - fk2)
                if dist_initState < (2 * r_body):
                    raise CaseNotFeasibleError("Initial configuration in self collision")
        if checkGoalReachible:
            if np.linalg.norm(np.array(self._params["goal"])) > self._params["n"]:
                raise CaseNotFeasibleError("Goal unreachible")

    def save(self, folderPath):
        expFilename = folderPath + "/exp.yaml"
        goalFilename = folderPath + "/goal.csv"
        initStateFilename = folderPath + "/initState.csv"
        obstFile = folderPath + "/obst"
        with open(expFilename, "w") as file:
            yaml.dump(self._params, file)
        for i, o in enumerate(self._obsts):
            o.toCSV(obstFile + "_" + str(i) + ".csv")
        with open(goalFilename, "w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            csv_writer.writerow(self._params["goal"])
        with open(initStateFilename, "w") as file:
            csv_writer = csv.writer(file, delimiter=",")
            csv_writer.writerow(self._params["q0"])
