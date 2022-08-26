import subprocess
import os
import re

from plannerbenchmark.generic.experiment import Experiment

pkg_path = os.path.dirname(__file__) + "/../postProcessing/"


class SeriesPlotting(object):
    """SeriesPlotting compares different planners on the same experiments."""

    def __init__(self, folder: str, nbMetrics: int):
        self._folder: str = folder
        self._nbMetrics: int = nbMetrics

    def getPlannerNames(self) -> list:
        """Extracts the different planners present in the series."""
        plannerNames = set()
        pattern = re.compile(r"(\D*)_\d{8}_\d{6}")
        for fname in os.listdir(self._folder):
            match = re.match(pattern, fname)
            if match:
                plannerNames.add(match.group(1))
        return sorted(plannerNames)

    def plot(self) -> None:
        """Calls the script to generate the series plots."""
        plannerNames = self.getPlannerNames()
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/"
        curPath = pkg_path
        createPlotFolder = curPath + "plottingSeries"
        subprocess.Popen(
            [
                "./createPlot",
                self._folder,
            ] + plannerNames + 
            [str(self._nbMetrics)],
            cwd=createPlotFolder,
            stdout=subprocess.PIPE,
        ).wait()
