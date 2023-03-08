import os

from robotmpcs.models.mpcModel import MpcModel
from robotmpcs.models.diff_drive_mpc_model import MpcDiffDriveModel

from plannerbenchmark.generic.experiment import Experiment


def createSolver(mpc: dict, robot: dict, debug: bool=False):
    setup = {}
    setup['robot'] = robot
    setup['mpc'] = mpc
    if setup['robot']['base_type'] == 'holonomic':
        mpc_model = MpcModel(initParamMap=True, **setup)
    elif setup['robot']['base_type'] == 'diffdrive':
        mpc_model = MpcDiffDriveModel(initParamMap=True, **setup)
    mpc_model.setModel()
    mpc_model.setCodeoptions(debug=debug)
    location = f"{os.path.dirname(os.path.abspath(__file__))}/solverCollection/"
    mpc_model.generateSolver(location=location)
