import os
import pytest
import yaml


def test_resultDirExists():
    curDirName = os.path.dirname(os.path.abspath(__file__))
    resDirName = curDirName + '/../experiments/example/results'
    resultFolderFound = False
    for fname in os.listdir(resDirName):
        if os.path.isdir(os.path.join(resDirName,fname)):
            resultFolderFound = True
            resultFolder = os.path.join(resDirName, fname)
    # Check if folder time stamped folder exists
    assert resultFolderFound
    filenames = os.listdir(resultFolder)
    assert 'res.csv' in filenames
    assert 'planner.yaml' in filenames
    assert 'exp.yaml' in filenames

def test_yamlContents():
    curDirName = os.path.dirname(os.path.abspath(__file__))
    resDirName = curDirName + '/../experiments/example/results'
    for fname in os.listdir(resDirName):
        if os.path.isdir(os.path.join(resDirName,fname)):
            resultFolder = os.path.join(resDirName, fname)
    # verify planner config file
    plannerFile = os.path.join(resultFolder, 'planner.yaml')
    with open(plannerFile, 'r') as plannerStream:
        plannerDict = yaml.safe_load(plannerStream)
    expectedResultFolder = curDirName + '/../experiments/example/expectedResult/pdplanner_default'
    plannerDefaultFile = os.path.join(expectedResultFolder, 'planner.yaml')
    with open(plannerDefaultFile, 'r') as plannerStream:
        plannerDefaultDict = yaml.safe_load(plannerStream)
    assert plannerDict == plannerDefaultDict
    # verify exp config file
    expFile = os.path.join(resultFolder, 'exp.yaml')
    with open(expFile, 'r') as expStream:
        expDict = yaml.safe_load(expStream)
    expectedResultFolder = curDirName + '/../experiments/example/expectedResult/pdplanner_default'
    expDefaultFile = os.path.join(expectedResultFolder, 'exp.yaml')
    with open(expDefaultFile, 'r') as expStream:
        expDefaultDict = yaml.safe_load(expStream)
    assert expDict == expDefaultDict

