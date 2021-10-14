#! /usr/bin/python3
import argparse
import csv
import datetime
import time
import os
import sys
import warnings
from shutil import copyfile
import numpy as np

from fabricsExperiments.generic.experiment import Experiment
from fabricsExperiments.generic.fabricPlanner import FabricPlanner
from fabricsExperiments.generic.experimentSaver import ExperimentSaver


def blockPrint():
    sys.stdout = open(os.devnull, 'w')
    warnings.filterwarnings('ignore')


def enablePrint():
    sys.stdout = sys.__stdout__


class Runner(object):
    def __init__(self):
        print("Initializing Runner, TODO: Randomizing must be passed")
        self.initializeArgumentParser()

    def initializeArgumentParser(self):
        self._parser = argparse.ArgumentParser("Run motion planning experiment")
        self._parser.add_argument(
            "--caseSetup", "-case", type=str, help="Experiment setup file", required=True
        )
        self._parser.add_argument(
            "--planners", "-p", type=str, nargs="+", help="List of planner setup files", required=True
        )
        self._parser.add_argument(
            "--res-folder",
            "-res",
            type=str,
            default='results',
            help="Results folder",
        )
        self._parser.add_argument(
            "--numberRuns", "-n", type=int, default=1, help="Number of runs of the experiment"
        )
        self._parser.add_argument("--no-save", dest="save", action="store_false")
        self._parser.add_argument("--random-goal", dest="random_goal", action="store_true")
        self._parser.add_argument("--random-obst", dest="random_obst", action="store_true")
        self._parser.add_argument("--random-init", dest="random_init", action="store_true")
        self._parser.add_argument("--no-verbose", dest="verbose", action="store_false")
        self._parser.add_argument("--render", dest="render", action="store_true")
        self._parser.set_defaults(save=True)
        self._parser.set_defaults(random_goal=False)
        self._parser.set_defaults(random_obst=False)
        self._parser.set_defaults(random_init=False)
        self._parser.set_defaults(verbose=True)
        self._parser.set_defaults(render=False)

    def parseArguments(self):
        args = self._parser.parse_args()
        self._render = args.render
        self._save = args.save
        self._random_obst = args.random_obst
        self._random_init = args.random_init
        self._random_goal = args.random_goal
        self._numberRuns = args.numberRuns
        self._verbose = args.verbose
        self._render = args.render
        self._res_folder = args.res_folder
        self._planners = []
        self._experiment = Experiment(args.caseSetup)
        self._env = self._experiment.env()
        for i in range(0, len(args.planners), 2):
            plannerType = args.planners[i]
            plannerFile = args.planners[i+1]
            if plannerType == 'fabric':
                self._planners.append(FabricPlanner(self._experiment, plannerFile))

    def setPlanner(self):
        for planner in self._planners:
            planner.reset()
            planner.setJointLimits(self._experiment.limits())
            planner.setSelfCollisionAvoidance(self._experiment.rBody())
            planner.setObstacles(self._experiment.obstacles(), self._experiment.rBody())
            planner.setGoal(self._experiment.goal())
            planner.concretize()

    def run(self):
        for i in range(self._numberRuns):
            self._experiment.shuffle(self._random_obst, self._random_init, self._random_goal)
            self.setPlanner()
            q0, q0dot = self._experiment.initState()
            time.sleep(1)
            timeStamp = "{:%Y%m%d_%H%M%S}".format(datetime.datetime.now())
            for planner in self._planners:
                ob = self._env.reset(q0, q0dot)
                saver = ExperimentSaver(self._res_folder, timeStamp)
                saver.setSetups(self._experiment, planner)
                t = 0.0
                for i in range(self._experiment.T()):
                    q = ob[0:self._experiment.n()]
                    qdot = ob[self._experiment.n():2*self._experiment.n()]
                    t_before = time.time()
                    action = planner.computeAction(q, qdot)
                    solving_time = time.time() - t_before
                    saver.addResultPoint(t, q, qdot, solving_time)
                    ob, _, _, _ = self._env.step(action)
                    t += self._experiment.dt()
                    if self._render:
                        time.sleep(self._experiment.dt())
                        self._env.render()
                if self._save:
                    saver.save()

def newmain():
    myRunner = Runner()
    myRunner.parseArguments()
    myRunner.run()


if __name__ == "__main__":
    newmain()
