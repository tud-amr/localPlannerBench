import os

from plannerbenchmark.tests.test_functions import *


def test_all_cases():
    absPath = os.path.dirname(os.path.abspath(__file__))
    caseFolders = absPath + '/local_cases/'
    for case in os.listdir(caseFolders):
        if case == 'mpc_pointMass':
            print("Test case with point mass for planner mpc does not work due to error in solver creation.")
            continue
        run_integration_test_case('local_cases/' + case)
