import yaml
import os
import csv

from plannerbenchmark.postProcessing.caseEvaluation import CaseEvaluation


class SeriesEvaluation(object):

    """Evaluation class for a series"""

    def __init__(self, folder, recycle=False):
        self._folder = folder
        self._recycle = recycle

    def setMetrics(self, metricNames):
        self._metricNames = metricNames

    def process(self):
        listExpFolders = os.listdir(self._folder)
        totalExps = 0
        fullPaths = [self._folder + "/" + exp for exp in listExpFolders]
        self._kpis = {}
        for fullPath in fullPaths:
            if not os.path.isdir(fullPath):
                continue
            totalExps += 1
            case = CaseEvaluation(fullPath, recycle=self._recycle)
            case.setMetrics(self._metricNames)
            case.process()
            case.writeKpis()
            timeStamp = case.timeStamp()
            plannerName = case.plannerName()
            kpi = case.kpis(short=True)
            if not plannerName in self._kpis.keys():
                self._kpis[plannerName] = {}
            self._kpis[plannerName][timeStamp] = kpi

    def writeResults(self):
        self.writeKpis()
        self.writeResultTables()

    def writeKpis(self):
        postProcessFile = self._folder + "/postProcess.yaml"
        with open(postProcessFile, "w") as file:
            yaml.dump(self._kpis, file)

    def success(self, plannerName, timeStamp):
        return self._kpis[plannerName][timeStamp]['success']

    def filterKpis(self, kpiDict):
        kpiList = []
        for kpi, value in kpiDict.items():
            if not kpi == "success":
                kpiList.append(value)
        return kpiList

    def filterMetricNames(self):
        filteredMetricNames = ['timeStamp']
        for name in self._metricNames:
            if not name == "success":
                filteredMetricNames.append(name)
        return filteredMetricNames

    def writeResultTables(self):
        successes = {}
        for plannerKey in self._kpis:
            success = {-2: 0, -1:0, 1: 0}
            resultsTableFile = self._folder + "/resultsTable_" + plannerKey + ".csv"
            with open(resultsTableFile, "w") as file:
                csv_writer = csv.writer(file, delimiter=" ")
                csv_header = self.filterMetricNames()
                csv_writer.writerow(csv_header)
                for timeStampKey in self._kpis[plannerKey]:
                    success[self.success(plannerKey, timeStampKey)] += 1
                    if self.success(plannerKey, timeStampKey) > 0:
                        kpis_timeStamp = self.filterKpis(self._kpis[plannerKey][timeStampKey])
                        csv_writer.writerow([timeStampKey] + kpis_timeStamp)
            successes[plannerKey] = success
        successTableFile = self._folder + "/successTable.csv"
        with open(successTableFile, "w") as file:
            csv_writer = csv.writer(file, delimiter=" ")
            csv_header = ['planner', -2, -1, 1]
            csv_writer.writerow(csv_header)
            for planner in successes.keys():
                csv_writer.writerow(
                    [
                        planner,
                        successes[planner][-2],
                        successes[planner][-1],
                        successes[planner][1]
                    ]
                )
