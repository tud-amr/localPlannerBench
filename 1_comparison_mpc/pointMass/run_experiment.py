#! /usr/bin/python3
import argparse
import csv
import datetime
import time
import os
from shutil import copyfile

from fabricsExperiments.infrastructure.expSetup import ExpSetup
from fabrics.planner import FabricPlanner
from mpc.planner import MPCPlanner

class Experiment(object):

    def __init__(self, planner, setupFile, plannerSetup):
        self._setupFile = setupFile
        self._plannerSetup = plannerSetup
        self._setup = ExpSetup(setupFile)
        self._env = self._setup.makeEnv()
        if planner == 'fabric':
            self._planner = FabricPlanner(plannerSetup)
        elif planner == 'mpc':
            self._planner = MPCPlanner(plannerSetup)
        self._obsts = self._setup.obstacles()
        self._planner.addObstacles(self._obsts)
        self._planner.addGoal(self._setup.goal())

    def run(self):
        self._planner.concretize()
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
            a = self._planner.computeAction(q, qdot)
            t_planning = time.time() - t_before
            resDict = {'t': t, 't_planning': t_planning}
            for n_i in range(n):
                resDict['q' + str(n_i)] = q[n_i]
                resDict['q' + str(n_i) + 'dot'] = qdot[n_i]
                resDict['a' + str(n_i)] = q[n_i]
            self._res.append(resDict)
            ob, _, _, _ = self._env.step(a)
            t += self._env._dt

    def save(self, name, timeStamp):
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/results"
        if timeStamp == "":
            folderPath = curPath + "/" + name
        else:
            folderPath = curPath + "/" + name + "_" + timeStamp
        print("Saving results to : %s" % folderPath)
        os.mkdir(folderPath)
        resFile = folderPath + "/res.csv"
        obstFile = folderPath + "/obst"
        colNames = [*self._res[0]]
        with open(resFile, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=colNames)
            writer.writeheader()
            for res in self._res:
                writer.writerow(res)
        for i, o in enumerate(self._obsts):
            o.toCSV(obstFile + '_' + str(i) + '.csv')
        copyfile(self._setupFile, folderPath + "/exp.yaml")
        copyfile(self._plannerSetup, folderPath + "/planner.yaml")

def main():
    parser = argparse.ArgumentParser("Run motion planning experiment")
    parser.add_argument('setupFile', metavar="setup", type=str, help='setup file')
    parser.add_argument('planner', metavar="planner", type=str, help='planner (fabric, mpc)')
    parser.add_argument('plannerSetup', metavar="plannerSetup", type=str, help='planner setup')
    parser.add_argument('--output-file', '-o', type=str, default='output', help='Output filename without suffix', metavar='output')
    parser.add_argument('--no-stamp', dest='stamp', action='store_false')
    parser.set_defaults(stamp=True)
    args = parser.parse_args()
    thisExp = Experiment(args.planner, args.setupFile, args.plannerSetup)
    thisExp.run()
    if args.stamp:
        timeStamp = "{:%Y%m%d_%H%M%S}".format(datetime.datetime.now())
    else:
        timeStamp = ""
    thisExp.save(args.output_file, timeStamp)

if __name__ == "__main__":
    main()


