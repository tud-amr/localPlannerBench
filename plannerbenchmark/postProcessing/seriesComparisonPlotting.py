import subprocess
import os
import re

from plannerbenchmark.generic.experiment import Experiment
pkg_path = os.path.dirname(__file__) + '/../postProcessing/'


class SeriesComparisonPlotting(object):

    def __init__(self, folder, nbMetrics):
        self._folder = folder
        self._nbMetrics = nbMetrics

    def getPlannerNames(self):
        plannerNames = set()
        pattern = re.compile(r'(\D*)_\d{8}_\d{6}')
        for fname in os.listdir(self._folder):
            match = re.match(pattern, fname)
            if match:
                plannerNames.add(match.group(1))
        return list(plannerNames)

    def plot(self):
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/"
        curPath = pkg_path
        createPlotFolder = curPath + "plottingSeries"
        plannerNames = self.getPlannerNames()
        subprocess.Popen(
            [
                "./createComparisonPlot",
                self._folder,
                plannerNames[0],
                plannerNames[1],
                str(self._nbMetrics),
            ],
            cwd=createPlotFolder,
            stdout=subprocess.PIPE,
        ).wait()
