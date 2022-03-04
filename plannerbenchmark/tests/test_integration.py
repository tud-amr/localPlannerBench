import os
from subprocess import Popen, DEVNULL
import yaml
import numpy as np
import pytest


def verify_resultDirExists():
    resDirName = "tempResults"
    resultFolderFound = False
    for fname in os.listdir(resDirName):
        if os.path.isdir(os.path.join(resDirName, fname)):
            resultFolderFound = True
            resultFolder = os.path.join(resDirName, fname)
    # Check if folder time stamped folder exists
    assert resultFolderFound
    filenames = os.listdir(resultFolder)
    assert "res.csv" in filenames
    assert "planner.yaml" in filenames
    assert "exp.yaml" in filenames
    assert "plots" in filenames


def verify_configFiles(caseFolder):
    curDirName = os.path.dirname(os.path.abspath(__file__))
    resDirName = "tempResults"
    for fname in os.listdir(resDirName):
        if os.path.isdir(os.path.join(resDirName, fname)):
            resultFolder = os.path.join(resDirName, fname)
    # verify planner config file
    plannerFile = os.path.join(resultFolder, "planner.yaml")
    with open(plannerFile, "r") as plannerStream:
        plannerDict = yaml.safe_load(plannerStream)
    expectedResultFolder = curDirName + "/" + caseFolder + "/default_result"
    plannerDefaultFile = os.path.join(expectedResultFolder, "planner.yaml")
    with open(plannerDefaultFile, "r") as plannerStream:
        plannerDefaultDict = yaml.safe_load(plannerStream)
    assert plannerDict == plannerDefaultDict
    # verify exp config file
    expFile = os.path.join(resultFolder, "exp.yaml")
    with open(expFile, "r") as expStream:
        expDict = yaml.safe_load(expStream)
    expectedResultFolder = curDirName + "/" + caseFolder + "/default_result"
    expDefaultFile = os.path.join(expectedResultFolder, "exp.yaml")
    with open(expDefaultFile, "r") as expStream:
        expDefaultDict = yaml.safe_load(expStream)
    assert expDict == expDefaultDict


def verify_plotting():
    resDirName = "tempResults"
    for fname in os.listdir(resDirName):
        if os.path.isdir(os.path.join(resDirName, fname)):
            resultFolder = os.path.join(resDirName, fname)
    plotFolder = os.path.join(resultFolder, "plots")
    filenamesInPlot = os.listdir(plotFolder)
    assert "trajectory.eps" in filenamesInPlot


def verify_resFile(caseFolder):
    curDirName = os.path.dirname(os.path.abspath(__file__))
    resDirName = "tempResults"
    for fname in os.listdir(resDirName):
        if os.path.isdir(os.path.join(resDirName, fname)):
            resultFolder = os.path.join(resDirName, fname)
    resFileName = os.path.join(resultFolder, "res.csv")
    expectedResultFolder = curDirName + "/" + caseFolder + "/default_result"
    resDefaultFileName = os.path.join(expectedResultFolder, "res.csv")
    resArray = np.genfromtxt(resFileName, delimiter=",", skip_header=1)
    resDefaultArray = np.genfromtxt(resDefaultFileName, delimiter=",", skip_header=1)
    np.testing.assert_array_almost_equal(resArray[:, 2:], resDefaultArray[:, 2:])


def run_integration_test_case(caseFolder):
    print("\n------------------------------------------------------------------------")
    print(f"Running case from {caseFolder}")
    print("------------------------------------------------------------------------")
    tests_path = os.path.dirname(os.path.abspath(__file__))
    execPath = tests_path + "/../exec/"
    # run case
    cmdRunner = [
        "poetry",
        "run",
        execPath + "runner",
        "-c",
        tests_path + "/" + caseFolder + "/setup/exp.yaml",
        "-p",
        "pdplanner",
        tests_path + "/" + caseFolder + "/setup/planner.yaml",
        "--res-folder",
        "tempResults",
    ]
    Popen(cmdRunner, stdout=DEVNULL).wait()
    cmdPostProcessor = [
        "poetry",
        "run",
        execPath + "postProcessor",
        "--exp",
        "tempResults",
        "-k",
        "solverTime",
        "clearance",
        "time2Goal",
        "--plot",
        "--no-open",
        "--latest",
    ]
    Popen(cmdPostProcessor, stdout=DEVNULL).wait()
    # run integration test
    verify_resultDirExists()
    verify_configFiles(caseFolder)
    verify_resFile(caseFolder)
    verify_plotting()
    # clean up
    Popen(["rm", "-r", "-f", "tempResults"]).wait()


def test_all_cases():
    absPath = os.path.dirname(os.path.abspath(__file__))
    caseFolders = absPath + '/cases/'
    for case in os.listdir(caseFolders):
        run_integration_test_case('cases/' + case)
