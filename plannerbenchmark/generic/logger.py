import os
import sys
import yaml
import logging
import csv


class Logger(object):
    def __init__(self, resFolder, timeStamp):
        self._resFolder = resFolder
        self._timeStamp = timeStamp
        self._res = []

    def setSetups(self, exp, planner):
        self._exp = exp
        self._planner = planner

    def addResultPoint(self, t, q, qdot, action, solving_time, goal, obsts):
        resDict = {'t': t, 't_planning': solving_time}
        for a_i, a in enumerate(action):
            resDict['a' + str(a_i)] = a
        for n_i in range(self._exp.n() + 1):
            if n_i < self._exp.n():
                resDict['q' + str(n_i)] = q[n_i]
                resDict['q' + str(n_i) + 'dot'] = qdot[n_i]
            if self._exp.robotType() in ['panda', 'mobilePanda', 'tiago', 'albert']:
                fk = self._exp.fk(q, n_i, positionOnly=True)
                resDict['fk' + str(n_i) + "_x"] = fk[0]
                resDict['fk' + str(n_i) + "_y"] = fk[1]
                resDict['fk' + str(n_i) + "_z"] = fk[2]
            else:
                fk = self._exp.fk(q, n_i, positionOnly=True)
                resDict['fk' + str(n_i) + "_x"] = fk[0]
                resDict['fk' + str(n_i) + "_y"] = fk[1]
        for i_der, goal_der in enumerate(goal):
            for j_dim, goal_dim in enumerate(goal_der):
                resDict['goal_' + str(j_dim) + '_' + str(i_der)] = goal_dim
        for k_obst, obst in enumerate(obsts):
            for i_der, obst_der in enumerate(obst):
                for j_dim in range(obst_der.size):
                    resDict['obst_' + str(k_obst) + '_' + str(j_dim) + '_' + str(i_der)] = obst_der[j_dim]
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
        while os.path.isdir(folderPath):
            folderPath = folderPath[:-2] + str(int(folderPath[-2:]) + 1)
        logging.info(f"Saving results to : {folderPath}")
        os.makedirs(folderPath)
        self.saveResult(folderPath)
        self._planner.save(folderPath)
        self._exp.save(folderPath)
