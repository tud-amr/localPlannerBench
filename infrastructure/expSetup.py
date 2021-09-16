import yaml
import csv
import gym
import pointRobot
import numpy as np

from obstacle import Obstacle

class ExpSetup(object):
    """docstring for Setup"""
    def __init__(self, setupFile, randomGoal=False, randomObst=False):
        self._randomGoal = randomGoal
        self._randomObst = randomObst
        self._setupFile = setupFile
        self.processFile()

    def processFile(self):
        with open(self._setupFile, 'r') as stream:
            self._params = yaml.safe_load(stream)
        if self._randomGoal:
            self._params['goal'] = np.random.uniform(low=-5, high=5, size=(2)).tolist()
        if self._randomObst:
            self._params['obstacles'] = []
            for i in range(5):
                name = 'obst_' + str(i)
                pos = np.random.uniform(low=-5, high=5, size=(2)).tolist()
                r = np.random.uniform(low=0, high=2.5)
                obst = {name: name, 'x': pos, 'r': r}
                self._params['obstacles'].append(obst)

    def makeEnv(self):
        return gym.make(self._params['env'], dt=self._params['dt'])

    def initialState(self):
        return self._params['q0'], self._params['q0dot']

    def goal(self):
        return self._params['goal']

    def obstacles(self):
        self._obsts = []
        for o in self._params['obstacles']:
            self._obsts.append(Obstacle(o['x'], o['r']))
        return self._obsts 

    def n(self):
        return self._params['n']

    def T(self):
        return self._params['T']

    def checkFeasibility(self):
        for o in self._obsts:
            dist_goal = np.linalg.norm(np.array(o.x()) - np.array(self._params['goal']) )
            dist_initState = np.linalg.norm(np.array(o.x()) - np.array(self._params['q0']) )
            if dist_goal < o.r() or dist_initState < o.r():
                return False
        return True

    def save(self, folderPath):
        expFilename = folderPath + "/exp.yaml"
        goalFilename = folderPath + "/goal.csv"
        initStateFilename = folderPath + "/initState.csv"
        obstFile = folderPath + "/obst"
        with open(expFilename, 'w') as file:
            yaml.dump(self._params, file)
        for i, o in enumerate(self._obsts):
            o.toCSV(obstFile + "_" + str(i) + ".csv")
        with open(goalFilename, 'w') as file:
            csv_writer = csv.writer(file, delimiter=',')
            csv_writer.writerow(self._params['goal'])
        with open(initStateFilename, 'w') as file:
            csv_writer = csv.writer(file, delimiter=',')
            csv_writer.writerow(self._params['q0'])
