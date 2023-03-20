import subprocess
import os

from plannerbenchmark.generic.experiment import Experiment
pkg_path = os.path.dirname(__file__) + '/../postProcessing/'


class CasePlotting(object):

    """Wrapper to direct to the correct gnuplot script for plotting the results
    of the experiment to an appropriate format."""

    def __init__(self, folder: str):
        self._folder:       str = folder
        self._experiment:   Experiment = Experiment(self._folder + "/exp.yaml")

    def plot(self) -> None:
        """Call the correct gnuplot script based on the robot type.

        The gnuplot scripts are called using subprocess.Popen to avoid
        additional libraries. Depending on the robot type, the gnuplot scripts
        take a different number of arguments. Output from the gnuplot scripts
        is passed to the subprocess.PIPE.
        """
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/"
        curPath = pkg_path
        if 'point' in self._experiment.urdf_file():
            createPlotFolder = curPath + "plottingPointMass/"
            subprocess.Popen(
                ["./createPlot", self._folder],
                cwd=createPlotFolder,
                stdout=subprocess.PIPE,
            ).wait()
        elif 'panda' in self._experiment.urdf_file():
            createPlotFolder = curPath + "plottingPanda/"
            subprocess.Popen(
                ["./createPlot", self._folder],
                cwd=createPlotFolder,
            ).wait()
        elif 'boxer' in self._experiment.urdf_file():
            createPlotFolder = curPath + 'plottingGroundRobot/'
            subprocess.Popen(
                ["./createPlot", self._folder],
                cwd=createPlotFolder,
                stdout=subprocess.PIPE,
            ).wait()
