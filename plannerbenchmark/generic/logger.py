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

    def addResultPoint(self, t, observation, action, solving_time):
        q = observation['joint_state']['position']
        qdot = observation['joint_state']['velocity']
        resDict = {'t': t, 't_planning': solving_time}
        for a_i, a in enumerate(action):
            resDict['a' + str(a_i)] = a
        for n_i in range(self._exp.n() + 1):
            if n_i < self._exp.n():
                resDict['q' + str(n_i)] = q[n_i]
                resDict['q' + str(n_i) + 'dot'] = qdot[n_i]
        for link_name in self._exp._fk.robot.link_names():
            fk = self._exp.fk(q, link_name, positionOnly=True)
            resDict['fk' + link_name + "_x"] = fk[0]
            resDict['fk' + link_name + "_y"] = fk[1]
            resDict['fk' + link_name + "_z"] = fk[2]
        i_goal = -1
        for _, goal in observation['FullSensor']['goals'].items():
            i_goal += 1
            for j_dim in range(3):
                resDict['goal_' +str(i_goal) + "_" + str(j_dim) + '_0'] = goal['position'][j_dim]
        for i_obstacle, obstacle in enumerate(list(observation['FullSensor']['obstacles'].values())):
            for j_dim in range(3):
                resDict['obst_' + str(i_obstacle) + '_' + str(j_dim) + '_0'] = obstacle['position'][j_dim]
                resDict['obst_' + str(i_obstacle) + '_' + str(j_dim) + '_1'] = obstacle['velocity'][j_dim]
            resDict['obst_' + str(i_obstacle) + '_radius'] = obstacle['size'][0]
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
