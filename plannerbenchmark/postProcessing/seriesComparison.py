import yaml
import os
import re
import csv
import subprocess
import numpy as np

pkg_path = os.path.dirname(__file__) + "/../postProcessing/"

from plannerbenchmark.postProcessing.seriesEvaluation import SeriesEvaluation

class BaselineNotFoundException(Exception):
    pass


class SeriesComparison(SeriesEvaluation):
    """Series comparison between two planners."""

    def __init__(self, folder: str, baseline: str, recycle: bool = False):
        self._baseline: str = baseline
        super().__init__(folder, recycle=recycle)

    def getPlannerNames(self) -> list:
        """Gets planner names."""
        plannerNames = set()
        pattern = re.compile(r"(\D*)_\d{8}_\d{6}")
        number_cases = 0
        for fname in os.listdir(self._folder):
            match = re.match(pattern, fname)
            if match:
                plannerNames.add(match.group(1))
                number_cases += 1
        self._number_cases = number_cases/len(plannerNames)
        if self._baseline not in plannerNames:
            raise BaselineNotFoundException(f"Baseline {self._baseline} not evaluated. Evaluated planners are {plannerNames}")
        self._planner_names =sorted(list(plannerNames))

    def process(self) -> None:
        """Process series, writes results and compares different planners."""
        super().process()
        super().writeResults()
        self._comparisons = {}
        for planner_name in filter(lambda name: name != self._baseline, self._planner_names):
            self.compare(planner_name)

    def compare(self, planner_name: str) -> None:
        """Compares the performance of two planners.

        Two planners are compared by computing the ratio for all individual
        metrics for every experiment in the series.
        """
        self.readResults()
        commonTimeStamps = self.getCasesSolvedByBoth(planner_name)
        comparedResults = {}
        for timeStamp in commonTimeStamps:
            A = self._results[self._baseline][timeStamp]
            B = self._results[planner_name][timeStamp]
            #comparedResults[timeStamp] = (A - B)/(A + B)
            comparedResults[timeStamp] = B/A
        self._comparisons[planner_name] = comparedResults
        self.writeComparison(planner_name)

    def writeComparison(self, planner_name: str) -> None:
        """Writes comparison resultTable_comparison.csv-file."""
        resultsTableFile = f"{self._folder}/data/comparisons_{planner_name}.csv"
        with open(resultsTableFile, "w") as file:
            csv_writer = csv.writer(file, delimiter=" ")
            csv_header = self.filterMetricNames()
            csv_writer.writerow(csv_header)
            for time_stamp, kpis in self._comparisons[planner_name].items():
                csv_writer.writerow([time_stamp] + kpis.tolist())

    def getCasesSolvedByBoth(self, planner_name: str) -> list:
        """Gets timestamps of all cases that were solved by both solvers.

        Returns
        -------
        list of str
            List containing all time stamps as strings that were solved by both methods.

        """
        res0 = set(self._results[self._baseline])
        res1 = set(self._results[planner_name])
        commonTimeStamps = []
        for timeStamp in res0.intersection(res1):
            commonTimeStamps.append(timeStamp)
        return commonTimeStamps

    def readResults(self):
        """Reads results from previous evaluations."""
        self._results = {}
        for planner_name in self._planner_names:
            plannerDict = {}
            resultsTableFile = (
                self._folder + "/data/resultsTable_" + planner_name + ".csv"
            )
            with open(resultsTableFile, mode="r") as inp:
                reader = csv.reader(inp, delimiter=" ")
                next(reader)
                for rows in reader:
                    values = np.array([float(x) for x in rows[1:]])
                    plannerDict[rows[0]] = values
            self._results[planner_name] = plannerDict

    def plot(self) -> None:
        """Call the correct gnuplot script.

        The gnuplot scripts are called using subprocess.Popen to avoid
        additional libraries. Depending on the robot type, the gnuplot scripts
        take a different number of arguments. Output from the gnuplot scripts
        is passed to the subprocess.PIPE.
        """
        super().plot()
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/"
        curPath = pkg_path
        createPlotFolder = curPath + "plottingSeries"
        for planner_name in filter(lambda name: name != self._baseline, self._planner_names):
            subprocess.Popen(
                [
                    "gnuplot",
                    "-c",
                    "makeComparisonPlot.gpi",
                    f"{self._folder}/",
                    self._baseline,
                    planner_name,
                ],
                cwd=createPlotFolder,
                stdout=subprocess.PIPE,
            ).wait()
        subprocess.Popen(
            [
                "gnuplot",
                "-c",
                "makeSuccessPlot.gpi",
                f"{self._folder}/",
                str(self._number_cases),
            ],
            cwd=createPlotFolder,
            stdout=subprocess.PIPE,
        ).wait()
