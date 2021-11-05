import os
import sys
import yaml
import csv

class ExperimentSaver(object):
    def __init__(self, resFolder, timeStamp):
        self._resFolder = resFolder
        self._timeStamp = timeStamp
        self._res = []

    def setSetups(self, exp, planner):
        self._exp = exp
        self._planner = planner

    def addResultPoint(self, t, q, qdot, solving_time, goal):
        resDict = {'t': t, 't_planning': solving_time}
        for n_i in range(self._exp.n() + 1):
            if n_i < self._exp.n():
                resDict['q' + str(n_i)] = q[n_i]
                resDict['q' + str(n_i) + 'dot'] = qdot[n_i]
                resDict['a' + str(n_i)] = q[n_i]
            fk = self._exp.fk(q, n_i)
            resDict['fk' + str(n_i) + "_x"] = fk[0]
            resDict['fk' + str(n_i) + "_y"] = fk[1]
            if self._exp.robotType() == 'planarArm':
                resDict['fk' + str(n_i) + "_theta"] = fk[2]
            if self._exp.robotType() == 'panda':
                resDict['fk' + str(n_i) + "_z"] = fk[2]
        for i_der, goal_der in enumerate(goal):
            for j_dim, goal_dim in enumerate(goal_der):
                resDict['goal_' + str(j_dim) + '_' + str(i_der)] = goal_dim
        self._res.append(resDict)

    def saveResult(self, folderPath):
        resFile = folderPath + "/res.csv"
        colNames = [*self._res[0]]
        with open(resFile, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=colNames)
            writer.writeheader()
            for res in self._res:
                writer.writerow(res)

    def save(self):
        #curPath = os.path.dirname(os.path.abspath(__file__)) + "/" + self._resFolder
        curPath = self._resFolder
        folderPath = curPath + "/" + self._planner.plannerType() + "_" + self._timeStamp
        print("Saving results to : %s" % folderPath)
        os.makedirs(folderPath)
        self.saveResult(folderPath)
        self._planner.save(folderPath)
        self._exp.save(folderPath)
