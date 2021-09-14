import yaml
import gym
import pointRobot

from obstacle import Obstacle

class ExpSetup(object):
    """docstring for Setup"""
    def __init__(self, setupFile):
        self._setupFile = setupFile
        self.processFile()

    def processFile(self):
        with open(self._setupFile, 'r') as stream:
            self._params = yaml.safe_load(stream)
        print(self._params)

    def makeEnv(self):
        return gym.make(self._params['env'], dt=self._params['dt'])

    def initialState(self):
        return self._params['q0'], self._params['q0dot']

    def goal(self):
        return self._params['goal']

    def obstacles(self):
        obsts = []
        for o in self._params['obstacles']:
            obsts.append(Obstacle(o['x'], o['r']))
        return obsts 

    def n(self):
        return self._params['n']

    def T(self):
        return self._params['T']
