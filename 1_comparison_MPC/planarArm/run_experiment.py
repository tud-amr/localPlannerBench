#! /usr/bin/python3
import argparse
import csv
import datetime
import time
import os
import sys
import warnings
import shutil
import numpy as np
from shutil import copyfile

from fabricsExperiments.infrastructure.expSetup import ExpSetup, CaseNotFeasibleError
from fabrics.planner import FabricPlanner
from mpc.planner import MPCPlanner

from numpyFk import numpyFk

def blockPrint():
    sys.stdout = open(os.devnull, 'w')
    warnings.filterwarnings('ignore')

def enablePrint():
    sys.stdout = sys.__stdout__

class Experiment(object):

    def __init__(self, setup, mpcSetup, fabricSetup, render=False):
        self._mpcSetup = mpcSetup
        self._fabricSetup = fabricSetup
        self._setup = setup
        self._render = render
        self._env = self._setup.makeEnv()
        self._mpcPlanner = MPCPlanner(mpcSetup, self._setup.n())
        self._fabricPlanner = FabricPlanner(fabricSetup, self._setup.n())
        self._obsts = self._setup.getObstacles()
        self._mpcPlanner.addObstacles(self._obsts, self._setup._params['r_body'])
        self._mpcPlanner.addJointLimits(self._setup.lowerLimits(), self._setup.upperLimits())
        self._mpcPlanner.addSelfCollisionAvoidance(self._setup._params['r_body'])
        self._mpcPlanner.addGoal(self._setup.goal())
        self._fabricPlanner.addObstacles(self._obsts, self._setup._params['r_body'])
        self._fabricPlanner.addJointLimits(self._setup.lowerLimits(), self._setup.upperLimits())
        self._fabricPlanner.addSelfCollisionAvoidance(self._setup._params['r_body'])
        self._fabricPlanner.addGoal(self._setup.goal())

    def run(self, planner='mpc'):
        if planner == 'mpc':
            self._mpcPlanner.concretize()
        elif planner == 'fabric':
            self._fabricPlanner.concretize()
        q0, qdot0 = self._setup.initialState()
        ob = self._env.reset(q0, qdot0)
        t = 0.0
        n = self._setup.n()
        self._res = []
        for i in range(self._setup.T()):
            if i%1000 == 0: print('Timestep : %d' % i)
            q = ob[0:n]
            qdot = ob[n:2*n]
            t_before = time.time()
            if planner == 'mpc':
                a = self._mpcPlanner.computeAction(q, qdot)
            elif planner == 'fabric':
                a = self._fabricPlanner.computeAction(q, qdot)
            t_planning = time.time() - t_before
            resDict = {'t': t, 't_planning': t_planning}
            for n_i in range(n+1):
                if n_i < n:
                    resDict['q' + str(n_i)] = q[n_i]
                    resDict['q' + str(n_i) + 'dot'] = qdot[n_i]
                    resDict['a' + str(n_i)] = q[n_i]
                fk = numpyFk(q, n_i)
                resDict['fk' + str(n_i) + "_x"] = fk[0]
                resDict['fk' + str(n_i) + "_y"] = fk[1]
                resDict['fk' + str(n_i) + "_theta"] = fk[2]
            self._res.append(resDict)
            ob, _, _, _ = self._env.step(a)
            if self._render:
                time.sleep(self._env._dt)
                self._env.render()
            t += self._env._dt
        return 0

    def save(self, timeStamp, errFlag, planner='mpc', resFolder='results'):
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/" + resFolder
        if timeStamp == "":
            folderPath = curPath + "/" + planner
        else:
            folderPath = curPath + "/" + planner + "_" + timeStamp
        print("Saving results to : %s" % folderPath)
        if not os.path.exists(folderPath):
            os.makedirs(folderPath)
        else:
            print("Overwriting %s" % folderPath)
            shutil.rmtree(folderPath)
            os.mkdir(folderPath)
        if errFlag >= 0:
            resFile = folderPath + "/res.csv"
            colNames = [*self._res[0]]
            with open(resFile, 'w') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=colNames)
                writer.writeheader()
                for res in self._res:
                    writer.writerow(res)
        self._setup.save(folderPath)
        if planner == 'fabric':
            copyfile(self._fabricSetup, folderPath + "/planner.yaml")
        elif planner == 'mpc':
            copyfile(self._mpcSetup, folderPath + "/planner.yaml")

def main():
    parser = argparse.ArgumentParser("Run motion planning experiment")
    parser.add_argument(
        "--caseSetup", "-case", type=str, help="setup file"
    )
    parser.add_argument(
        "--mpcSetup", "-mpc", type=str, help="mpc setup"
    )
    parser.add_argument(
        "--fabricSetup", "-fab", type=str, help="fabric setup"
    )
    parser.add_argument(
        "--res-folder",
        "-res",
        type=str,
        default='results',
        help="Results folder",
    )
    parser.add_argument("--no-stamp", dest="stamp", action="store_false")
    parser.add_argument("--random-goal", dest="random_goal", action="store_true")
    parser.add_argument("--random-obst", dest="random_obst", action="store_true")
    parser.add_argument("--random-init", dest="random_init", action="store_true")
    parser.add_argument("--no-verbose", dest="verbose", action="store_false")
    parser.add_argument("--render", dest="render", action="store_true")
    parser.set_defaults(stamp=True)
    parser.set_defaults(render=False)
    parser.set_defaults(random_goal=False)
    parser.set_defaults(random_obst=False)
    parser.set_defaults(random_init=False)
    parser.set_defaults(verbose=True)
    args = parser.parse_args()
    if args.stamp:
        timeStamp = "{:%Y%m%d_%H%M%S}".format(datetime.datetime.now())
    else:
        timeStamp = ""
    try:
        setup = ExpSetup(
            args.caseSetup,
            randomGoal=args.random_goal,
            randomObst=args.random_obst,
            randomInit=args.random_init,
        )
        setup.checkFeasibility(checkGoalReachible=True)
    except CaseNotFeasibleError as e:
        print("ERROR: %s" % e)
        return 
    if not args.verbose:
        blockPrint()
    thisExp = Experiment(setup, args.mpcSetup, args.fabricSetup, render=args.render)
    errFlag = thisExp.run(planner="fabric")
    thisExp.save(timeStamp, errFlag, planner="fabric", resFolder=args.res_folder)
    errFlag = thisExp.run(planner="mpc")
    thisExp.save(timeStamp, errFlag, planner="mpc", resFolder=args.res_folder)

if __name__ == "__main__":
    main()


