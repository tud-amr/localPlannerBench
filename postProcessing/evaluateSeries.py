import sys
import os
import yaml
import pprint
import numpy as np


class SeriesResult(object):
    def __init__(self, seriesFolder):
        self._seriesFolder = seriesFolder
        self._res = {}

    def getExpResults(self, expFolder):
        yamlFile = expFolder + "/postProcess.yaml"
        with open(yamlFile, "r") as stream:
            resDict = yaml.safe_load(stream)
        return resDict

    def getSeriesResults(self):
        expList = [
            os.path.join(self._seriesFolder, o) for o in os.listdir(self._seriesFolder)
        ]
        for exp in expList:
            timeStamp = exp[-15:]
            method = "fabric" if "fabric" in exp else "mpc"
            if not timeStamp in self._res.keys():
                self._res[timeStamp] = {}
            self._res[timeStamp][method] = self.getExpResults(exp)

    def evaluateSeries(self):
        self._series_res = {}
        clearances = {"mpc": [], "fabric": []}
        selfClearances = {"mpc": [], "fabric": []}
        pathLengths = {"mpc": [], "fabric": []}
        time2Goals = {"mpc": [], "fabric": []}
        successes = {"mpc": [], "fabric": []}
        feasibleCases = {'mpc': [], 'fabric': []}
        solverTimes = {'mpc': [], 'fabric': []}
        self._n = len(self._res.keys())
        for exp in self._res.keys():
            for method in self._res[exp].keys():
                clearances[method].append(
                    self._res[exp][method]["clearance"]["minDist"]
                )
                # selfClearances[method].append(self._res[exp][method]['selfClearance']['minDist'])
                pathLengths[method].append(self._res[exp][method]["pathLength"])
                time2Goals[method].append(self._res[exp][method]["time2Goal"][1])
                successes[method].append(self._res[exp][method]["success"][0])
                feasibleCases[method].append(self._res[exp][method]['success'][0])
                solverTimes[method].append(self._res[exp][method]['solvertime'])
        #pprint.pprint(successes)
        #pprint.pprint(pathLengths)
        #pprint.pprint(time2Goals)
        #pprint.pprint(clearances)
        self._series_res["clearance"] = np.array(
            clearances["fabric"]
        ) / np.array(clearances["mpc"])
        self._series_res["pathLengths"] = np.array(pathLengths["fabric"]) / np.array(
            pathLengths["mpc"]
        )
        self._series_res["time2Goals"] = np.array(time2Goals["fabric"]) / np.array(
            time2Goals["mpc"]
        )
        self._series_res['feasibleCases'] = feasibleCases
        __import__('pdb').set_trace()
        self._series_res["success"] = (
                sum(successes["mpc"])/self._n, 
                sum(successes["fabric"])/self._n, 
            )
        self._series_res['solverTimes'] = np.array(solverTimes['fabric']) / np.array(
            solverTimes['mpc']
        )

    def getKPIs(self):
        kpis = {}
        feasibleCases = [i for i in range(self._n) if (self._series_res['feasibleCases']['mpc'][i] and self._series_res['feasibleCases']['fabric'])]
        kpis['clearance[fabric/mpc]'] = np.mean(self._series_res['clearance'][feasibleCases])
        kpis['pathLength[fabric/mpc]'] = np.mean(self._series_res['pathLengths'][feasibleCases])
        kpis['solverTimes[fabric/mpc]'] = np.mean(self._series_res['solverTimes'][feasibleCases])
        kpis['time2Goals[fabric/mpc]'] = np.mean(self._series_res['time2Goals'][feasibleCases])
        kpis['success[mpc, fabric]'] = self._series_res['success']
        return kpis


if __name__ == "__main__":
    seriesFolder = sys.argv[1]
    seriesRes = SeriesResult(seriesFolder)
    seriesRes.getSeriesResults()
    seriesRes.evaluateSeries()
    kpis = seriesRes.getKPIs()
    pprint.pprint(kpis)
