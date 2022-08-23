import csv
import logging
import numpy as np
import yaml
import re

from plannerbenchmark.generic.experiment import Experiment
from plannerbenchmark.postProcessing.helpers import createMetricsFromNames
from plannerbenchmark.postProcessing.metrics import SuccessMetric


class CaseEvaluation(object):

    """Evaluation class for a single case"""

    def __init__(self, folder: str, recycle: bool = False):
        """Constructor for post processing of single experimental case.

        Parameters
        ----------
        folder : str
            Full path to experiment folder.
        recycle : bool, optional
            Flag that specifies whether previous postProcessing should be recycled.
            (False by default)
        """
        self._folder = folder
        self._recycle = recycle
        self.decodeFolderName()
        if self._recycle:
            self.readResults()
        else:
            self._metrics = []
            self._experiment = Experiment(self._folder + "/exp.yaml")

    def interval(self) -> int:
        """Gets time interval of the planner in this experiment."""
        plannerFile = self._folder + "/planner.yaml"
        with open(plannerFile, "r") as file:
            self._plannerSetup = yaml.safe_load(file)
        return self._plannerSetup["interval"]

    def decodeFolderName(self) -> None:
        """Decodes the folder name into planner and timeStamp using regex."""
        pattern = re.compile(r".*\/(.*)_(\d{8}_\d{6})")
        match = re.match(pattern, self._folder)
        self._plannerName = match.group(1)
        self._timeStamp = match.group(2)

    def timeStamp(self) -> str:
        """Gets time stamp of the experiment as string."""
        return self._timeStamp

    def plannerName(self) -> str:
        """Gets planner name  of the experiment as string."""
        return self._plannerName

    def experiment(self) -> Experiment:
        """Gets expermient instance of the experiment."""
        return self._experiment

    def setMetrics(self, metricNames: list) -> None:
        """Sets the metrics for case evaluation.

        Parameters
        ----------
        metricNames : list
            List of metrics that should be evaluated.
        """
        if not self._recycle:
            self._metrics = createMetricsFromNames(
                metricNames, self._experiment, interval=self.interval()
            )

    def readData(self) -> None:
        """Reads in results csv-file as dictonary."""
        fileName = self._folder + "/res.csv"
        rawData = np.genfromtxt(fileName, delimiter=",", skip_header=1).transpose()
        with open(fileName) as csvfile:
            reader = csv.reader(csvfile, delimiter=",")
            header = next(reader)
        self._res = {}
        for i, key in enumerate(header):
            self._res[key] = rawData[i]

    def evaluateMetrics(self) -> None:
        """Evaluates all metrics specified for this experiment.

        The evaulations of the metrics are stored in the self._kpis didctonary.
        Note that all metrics evaluations are dictonaries themselves with a
        required key `short` that summarizes the result of this metric.

        """
        self._kpis = {}
        for metric in self._metrics:
            self._kpis[metric.name()] = metric.computeMetric(self._res)

    def evaluateSuccess(self) -> None:
        """Evaluatios whether the planning problem was sucessfully solved.

        A problem was sucessfully solved if the goal was reached and no
        collision occured during execution (minClearance > 0).
        """
        try:
            successMetric = SuccessMetric(
                "success",
                [],
                {
                    "minClearance": self._kpis["clearance"]["short"],
                    "reachingFlag": self._kpis["time2Goal"]["short"],
                },
            )
            self._kpis["success"] = successMetric.computeMetric(self._res)
        except KeyError:
            logging.info("Reaching or clearance metric not found: Success assumed.")
            self._kpis["success"] = {'short': 1}

    def process(self) -> None:
        """Processing the experiment."""
        if self._recycle:
            return
        self.readData()
        self.evaluateMetrics()
        self.evaluateSuccess()

    def readResults(self) -> None:
        """Read results from previous evaluations."""
        postProcessFile = self._folder + "/postProcess.yaml"
        with open(postProcessFile, "r") as file:
            self._kpis = yaml.safe_load(file)

    def kpis(self, short: bool = False) -> dict:
        """Gets key performance indicators.

        Parameters
        ----------
        short : bool, optional
            Flag specifying whether only the kpi summaries should be returned.
            (by default the detailed description is returned)
        """
        if short:
            shortKpis = {}
            for key in self._kpis:
                shortKpis[key] = self._kpis[key]["short"]
            return shortKpis
        return self._kpis

    def writeResults(self) -> None:
        """Writes results to yaml files."""
        self.writeKpis()

    def writeKpis(self) -> None:
        """Writes kpis to postProcess.yaml file."""
        postProcessFile = self._folder + "/postProcess.yaml"
        with open(postProcessFile, "w") as file:
            yaml.dump(self._kpis, file)
