import subprocess
import os
import re

from plannerbenchmark.generic.experiment import Experiment

pkg_path = os.path.dirname(__file__) + "/../postProcessing/"


class SeriesComparisonPlotting(object):
    """Plotting wrapper for series comparisons."""

    def __init__(self, folder: str, nbMetrics: int):
        self._folder: str = folder
        self._nbMetrics: int = nbMetrics

    def getPlannerNames(self) -> list:
        """Gets planner names."""
        plannerNames = set()
        pattern = re.compile(r"(\D*)_\d{8}_\d{6}")
        for fname in os.listdir(self._folder):
            match = re.match(pattern, fname)
            if match:
                plannerNames.add(match.group(1))
        return sorted(list(plannerNames))

    def plot(self) -> None:
        """Call the correct gnuplot script.

        The gnuplot scripts are called using subprocess.Popen to avoid
        additional libraries. Depending on the robot type, the gnuplot scripts
        take a different number of arguments. Output from the gnuplot scripts
        is passed to the subprocess.PIPE.
        """
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
