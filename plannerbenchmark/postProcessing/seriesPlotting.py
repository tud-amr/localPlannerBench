import subprocess
import os

from plannerbenchmark.generic.experiment import Experiment
pkg_path = os.path.dirname(__file__) + '/../postProcessing/'


class SeriesPlotting(object):

    def __init__(self, folder, nbMetrics):
        self._folder = folder
        self._nbMetrics = nbMetrics

    def plot(self):
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/"
        curPath = pkg_path
        createPlotFolder = curPath + "plottingSeries"
        subprocess.Popen(
            [
                "./createPlot",
                self._folder,
                "fabric",
                str(self._nbMetrics),
            ],
            cwd=createPlotFolder,
            stdout=subprocess.PIPE,
        ).wait()
