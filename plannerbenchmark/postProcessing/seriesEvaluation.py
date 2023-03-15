import os
import re
import logging
import subprocess
import csv
import yaml

from plannerbenchmark.postProcessing.caseEvaluation import CaseEvaluation

pkg_path = os.path.dirname(__file__) + "/../postProcessing/"


class SeriesEvaluation(object):

    """Evaluation class for a series

    Attributes
    ----------

    _folder: str
        Full path to the series folder.
    _recycle: bool
        Flag that tell the evaluation if old evaluation can be recycled.
        (by default this is set to false)
    """

    def __init__(self, folder: str, recycle: bool = False):
        self._folder: str = folder
        self._recycle: bool = recycle
        os.makedirs(f"{self._folder}/plot", exist_ok=True)
        os.makedirs(f"{self._folder}/data", exist_ok=True)
        self.getPlannerNames()

    def getPlannerNames(self) -> list:
        """Extracts the different planners present in the series."""
        plannerNames = set()
        pattern = re.compile(r"(\D*)_\d{8}_\d{6}")
        for fname in os.listdir(self._folder):
            match = re.match(pattern, fname)
            if match:
                plannerNames.add(match.group(1))
        self._planner_names = sorted(plannerNames)

    def setMetrics(self, metricNames: list) -> None:
        self._metricNames = metricNames

    def process(self) -> None:
        """Performs post postprocessing for series of experiments.

        For every experiment folder inside the series folder, a case
        evaluation is performed. Then, a summary of all cases is composed
        in this class.
        """
        listExpFolders = os.listdir(self._folder)
        totalExps = 0
        fullPaths = [self._folder + "/" + exp for exp in listExpFolders]
        self._kpis = {}
        for fullPath in fullPaths:
            if not os.path.isdir(fullPath):
                continue
            if fullPath[-4:] in ['data', 'plot']:
                continue
            logging.info(f"Evaluating experiment from folder : {fullPath}")
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

    def writeResults(self) -> None:
        """Writes results to yaml files."""
        self.writeKpis()
        self.writeResultTables()

    def writeKpis(self) -> None:
        """Writes kpis to postProcess.yaml file."""
        postProcessFile = self._folder + "/postProcess.yaml"
        with open(postProcessFile, "w") as file:
            yaml.dump(self._kpis, file)

    def success(self, plannerName: str, timeStamp: str) -> dict:
        """Gets evaluation of success metric."""
        return self._kpis[plannerName][timeStamp]["success"]

    def filterKpis(self, kpiDict: dict) -> list:
        """Transforms kpis to list and excludes success metric."""
        kpiList = []
        for kpi, value in kpiDict.items():
            if not kpi == "success":
                kpiList.append(value)
        return kpiList

    def filterMetricNames(self) -> list:
        """Filters metric names to exclude success metric name."""
        filteredMetricNames = ["timeStamp"]
        for name in self._metricNames:
            if not name == "success":
                filteredMetricNames.append(name)
        return filteredMetricNames

    def writeResultTables(self) -> None:
        """Write result table to successTable.csv-file."""
        successes = {}
        for plannerKey in self._kpis:
            success = {-2: 0, -1: 0, 1: 0}
            resultsTableFile = self._folder + "/data/resultsTable_" + plannerKey + ".csv"
            with open(resultsTableFile, "w") as file:
                csv_writer = csv.writer(file, delimiter=" ")
                csv_header = self.filterMetricNames()
                csv_writer.writerow(csv_header)
                for timeStampKey in self._kpis[plannerKey]:
                    success[self.success(plannerKey, timeStampKey)] += 1
                    if self.success(plannerKey, timeStampKey) > 0:
                        kpis_timeStamp = self.filterKpis(
                            self._kpis[plannerKey][timeStampKey]
                        )
                        csv_writer.writerow([timeStampKey] + kpis_timeStamp)
            successes[plannerKey] = success
        successTableFile = self._folder + "/data/successTable.csv"
        with open(successTableFile, "w") as file:
            csv_writer = csv.writer(file, delimiter=" ")
            csv_header = ["planner", -2, -1, 1]
            csv_writer.writerow(csv_header)
            for planner in sorted(successes.keys()):
                csv_writer.writerow(
                    [
                        planner,
                        successes[planner][-2],
                        successes[planner][-1],
                        successes[planner][1],
                    ]
                )

    def plot(self) -> None:
        """Calls the script to generate the series plots."""
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/"
        curPath = pkg_path
        createPlotFolder = curPath + "plottingSeries"
        for planner_name in self._planner_names:
            subprocess.Popen(
                [
                    "gnuplot",
                    "-c",
                    "makeSeriesPlot.gpi",
                    f"{self._folder}/",
                    planner_name,
                ],
                cwd=createPlotFolder,
                stdout=subprocess.PIPE,
            ).wait()
