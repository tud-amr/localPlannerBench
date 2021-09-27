import yaml
import csv
import gym
import pointRobot
import nLinkReacher
import pandaReacher
import numpy as np

from obstacle import Obstacle

from numpyFk import numpyFk


class ExpSetup(object):
    """docstring for Setup"""

    def __init__(self, setupFile, randomGoal=False, randomObst=False):
        self._randomGoal = randomGoal
        self._randomObst = randomObst
        self._setupFile = setupFile
        self.processFile()

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

    def initialState(self):
        return self._params["q0"], self._params["q0dot"]

    def goal(self):
        return self._params["goal"]

    def obstacles(self):
        self._obsts = []
        for o in self._params["obstacles"]:
            self._obsts.append(Obstacle(o["x"], o["r"]))
        return self._obsts

    def n(self):
        return self._params["n"]

    def T(self):
        return self._params["T"]

    def lowerLimits(self):
        return self._params["limits"]["lower"]

    def upperLimits(self):
        return self._params["limits"]["upper"]

    def checkFeasibility(self):
        for o in self._obsts:
            for i in range(1, self._params["n"] + 1):
                fk = numpyFk(self._params["q0"], i)[0:2]
                dist_initState = np.linalg.norm(np.array(o.x()) - fk)
                if dist_initState < o.r():
                    print("Infeasible initial configuration")
                    return False
            dist_goal = np.linalg.norm(np.array(o.x()) - np.array(self._params["goal"]))
            if dist_goal < o.r():
                print("Goal in collision")
                return False
            if np.linalg.norm(np.array(self._params["goal"])) > self._params["n"]:
                print("Goal unreachible")
                return False
        return True

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
