import csv
import numpy as np
import yaml
import re

from plannerbenchmark.generic.experiment import Experiment
from plannerbenchmark.postProcessing.helpers import createMetricsFromNames
from plannerbenchmark.postProcessing.metrics import SuccessMetric


class CaseEvaluation(object):

    """Evaluation class for a single case"""

    def __init__(self, folder, recycle=False):
        self._folder = folder
        self._recycle = recycle
        self.decodeFolderName()
        if self._recycle:
            self.readResults()
        else:
            self._metrics = []
            self._experiment = Experiment(self._folder + "/exp.yaml")

    def decodeFolderName(self):
        pattern = re.compile(r'.*\/(.*)_(\d{8}_\d{6})')
        match = re.match(pattern, self._folder)
        self._plannerName = match.group(1)
        self._timeStamp = match.group(2)

    def experiment(self):
        return self._experiment

    def setMetrics(self, metricNames):
        if not self._recycle:
            self._metrics = createMetricsFromNames(metricNames, self._experiment)

    def readData(self):
        fileName = self._folder + "/res.csv"
        rawData = np.genfromtxt(fileName, delimiter=',', skip_header=1).transpose()
        with open(fileName) as csvfile:
            reader = csv.reader(csvfile, delimiter=",")
            header = next(reader)
        self._res = {}
        for i, key in enumerate(header):
            self._res[key] = rawData[i]

    def evaluateMetrics(self):
        self._kpis = {}
        for metric in self._metrics:
            self._kpis[metric.name()] = metric.computeMetric(self._res)

    def evaluateSuccess(self):
        successMetric = SuccessMetric(
            'success',
            [],
            {
                "minClearance": self._kpis['clearance']['short'], 
                "reachingFlag": self._kpis['time2Goal']['short']
            }
        )
        self._kpis['success'] = successMetric.computeMetric(self._res)
        return

    def process(self):
        if self._recycle:
            return
        self.readData()
        self.evaluateMetrics()
        self.evaluateSuccess()

    def readResults(self):
        postProcessFile = self._folder + "/postProcess.yaml"
        with open(postProcessFile, "r") as file:
            self._kpis = yaml.safe_load(file)

    def kpis(self, short=False):
        if short:
            shortKpis = {}
            for key in self._kpis:
                shortKpis[key] = self._kpis[key]['short']
            return shortKpis
        return self._kpis

    def writeResults(self):
        self.writeKpis()

    def writeKpis(self):
        postProcessFile = self._folder + "/postProcess.yaml"
        with open(postProcessFile, "w") as file:
            yaml.dump(self._kpis, file)

    def timeStamp(self):
        return self._timeStamp

    def plannerName(self):
        return self._plannerName
