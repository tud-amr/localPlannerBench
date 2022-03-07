import yaml
import os
import re
import csv
import numpy as np

from plannerbenchmark.postProcessing.seriesEvaluation import SeriesEvaluation


class SeriesComparison(SeriesEvaluation):

    def getPlannerNames(self):
        plannerNames = set()
        pattern = re.compile(r'(\D*)_\d{8}_\d{6}')
        for fname in os.listdir(self._folder):
            match = re.match(pattern, fname)
            if match:
                plannerNames.add(match.group(1))
        return list(plannerNames)

    def process(self):
        super().process()
        self.compare()
    
    def compare(self):
        self.readResults()
        commonTimeStamps = self.getCasesSolvedByBoth()
        comparedResults = {}
        for timeStamp in commonTimeStamps:
            comparedResults[timeStamp] = self._results[0][timeStamp]/self._results[1][timeStamp]
        self._results.append(comparedResults)
        self.writeComparison()

    def writeComparison(self):
        resultsTableFile = self._folder + "/resultsTable_comparison.csv"
        with open(resultsTableFile, "w") as file:
            csv_writer = csv.writer(file, delimiter=" ")
            csv_header = self.filterMetricNames()
            csv_writer.writerow(csv_header)
            for timeStampKey in self._results[2]:
                kpis_timeStamp = self._results[2][timeStampKey].tolist()
                csv_writer.writerow([timeStampKey] + kpis_timeStamp)


    def getCasesSolvedByBoth(self):
        res0 = set(self._results[0])
        res1 = set(self._results[1])
        commonTimeStamps = []

        for timeStamp in res0.intersection(res1):
            commonTimeStamps.append(timeStamp)
        return commonTimeStamps

    def readResults(self):
        plannerNames = self.getPlannerNames()
        self._results = []
        for i in range(2):
            plannerDict = {}
            resultsTableFile = self._folder + "/resultsTable_" + plannerNames[i] + ".csv"
            with open(resultsTableFile, mode='r') as inp:
                reader = csv.reader(inp, delimiter=' ')
                next(reader)
                for rows in reader:
                    values = np.array([float(x) for x in rows[1:]])
                    plannerDict[rows[0]] = values
            self._results.append(plannerDict)
