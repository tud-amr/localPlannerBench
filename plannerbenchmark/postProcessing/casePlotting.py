import subprocess
import os

from plannerbenchmark.generic.experiment import Experiment
pkg_path = os.path.dirname(__file__) + '/../postProcessing/'


class CasePlotting(object):

    """Docstring for CasePlotting. """

    def __init__(self, folder):
        self._folder = folder
        self._experiment = Experiment(self._folder + "/exp.yaml")

    def plot(self):
        curPath = os.path.dirname(os.path.abspath(__file__)) + "/"
        curPath = pkg_path
        if self._experiment.robotType() == "planarArm":
            createPlotFolder = curPath + "plottingPlanarArm"
            subprocess.Popen(
                [
                    "./createPlot",
                    self._folder,
                    str(self._experiment.n())
                ],
                cwd=createPlotFolder,
                stdout=subprocess.PIPE,
            ).wait()
        elif self._experiment.robotType() == "pointRobot":
            createPlotFolder = curPath + "plottingPointMass/"
            subprocess.Popen(
                ["./createPlot", self._folder],
                cwd=createPlotFolder,
                stdout=subprocess.PIPE,
            ).wait()
        elif self._experiment.robotType() == "panda":
            createPlotFolder = curPath + "plottingPanda/"
            subprocess.Popen(
                ["./createPlot", self._folder],
                cwd=createPlotFolder,
            ).wait()
        elif self._experiment.robotType() == 'groundRobot':
            createPlotFolder = curPath + 'plottingGroundRobot/'
            subprocess.Popen(
                ["./createPlot", self._folder],
                cwd=createPlotFolder,
                stdout=subprocess.PIPE,
            ).wait()
        elif self._experiment.robotType() == 'boxer':
            createPlotFolder = curPath + 'plottingGroundRobot/'
            subprocess.Popen(
                ["./createPlot", self._folder],
                cwd=createPlotFolder,
                stdout=subprocess.PIPE,
            ).wait()
