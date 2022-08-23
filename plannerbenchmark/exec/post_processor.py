#!/usr/bin/env python3
import argparse
import os
import logging
import subprocess

from plannerbenchmark.postProcessing.caseEvaluation import CaseEvaluation
from plannerbenchmark.postProcessing.seriesEvaluation import SeriesEvaluation
from plannerbenchmark.postProcessing.seriesComparison import SeriesComparison
from plannerbenchmark.postProcessing.casePlotting import CasePlotting
from plannerbenchmark.postProcessing.seriesPlotting import SeriesPlotting
from plannerbenchmark.postProcessing.helpers import createMetricsFromNames
from plannerbenchmark.postProcessing.seriesComparisonPlotting import SeriesComparisonPlotting

log_levels = {"WARNING": 30, "INFO": 20, "DEBUG": 10, "QUIET": 100}

class SlimPostProcessor(object):

    """Docstring for SlimPostProcessor. """

    def __init__(self):
        self._parser = argparse.ArgumentParser(
            "Run post processing for motion planning experiment"
        )
        self._parser.add_argument(
            "--expFolder",
            "-exp",
            type=str,
            help="Experiment result folder",
            required=True,
        )
        self._parser.add_argument(
            "--kpis", "-k",
            type=str,
            nargs="+",
            help="List of all kpis/metrics to be evaluated",
            required=True
        )
        self._parser.add_argument(
                "--log-level",
                "-v",
                "-ll",
                type=str,
                default="INFO",
                help="Set logging level (Choose between DEBUG, INFO, WARNING, QUIET)"
        )
        self._parser.add_argument("--latest", dest="latest", action="store_true")
        self._parser.set_defaults(latest=False)
        self._parser.add_argument("--series", dest="series", action="store_true")
        self._parser.add_argument("--plot", dest="plot", action="store_true")
        self._parser.add_argument("--recycle", dest="recycle", action="store_true")
        self._parser.add_argument("--compare", dest="compare", action="store_true")
        self._parser.add_argument("--open", dest="open", action="store_true")
        self._parser.set_defaults(series=False, plot=False, recycle=False, compare=False, open=False)

    def run(self):
        args = self._parser.parse_args()
        logging.basicConfig(level=log_levels[args.log_level])
        kpis = sorted(args.kpis)
        nbMetrics = len(kpis)
        folder = os.getcwd() + "/" + args.expFolder
        if args.latest:
            import glob
            file_type = r'/*'
            files = glob.glob(folder + file_type)
            if not files:
                raise FileNotFoundError(folder)
            folder = max(files, key=os.path.getctime)


        if args.series:
            evaluator = SeriesEvaluation(folder, recycle=args.recycle)
            if args.compare:
                evaluator = SeriesComparison(folder, recycle=args.recycle)
        else:
            evaluator = CaseEvaluation(folder, recycle=args.recycle)
        evaluator.setMetrics(kpis)
        evaluator.process()
        evaluator.writeResults()
        if args.plot and not args.series:
            plotter = CasePlotting(folder)
            plotter.plot()
        if args.plot and args.series:
            plotter = SeriesPlotting(folder, nbMetrics)
            plotter.plot()
            if args.compare:
                comparePlotter = SeriesComparisonPlotting(folder, nbMetrics)
                comparePlotter.plot()
        if args.open:
            subprocess.Popen(["xdg-open", folder], stdout=subprocess.PIPE)


def main():
    pp = SlimPostProcessor()
    pp.run()
