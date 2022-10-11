import os
import logging

from robotmpcs.models.pointRobotMpcModel import PointRobotMpcModel
from robotmpcs.models.planarArmMpcModel import PlanarArmMpcModel
from robotmpcs.models.boxerMpcModel import BoxerMpcModel
from robotmpcs.models.pandaMpcModel import PandaMpcModel


def createSolver(N=30, dt=0.1, robot_type="pointRobot", slack=False, n: int = 2):
    debug = False
    if robot_type == 'planarArm':
        mpcModel = PlanarArmMpcModel(2, N, n)
    elif robot_type == 'pointRobot':
        n = 2
        mpcModel = PointRobotMpcModel(2, N)
    elif robot_type == 'panda':
        n = 7
        mpcModel = PandaMpcModel(3, N)
    elif robot_type == 'boxer':
        n = 2
        mpcModel = BoxerMpcModel(N)
    else:
        logging.warn(f"Solver for robot type {robot_type} not available yet")
        
    if slack:
        mpcModel.setSlack()
    mpcModel.setDt(dt)
    mpcModel.setObstacles(5, mpcModel._m, inCostFunction=True)
    mpcModel.setModel()
    mpcModel.setCodeoptions(debug=debug)
    location = f"{os.path.dirname(os.path.abspath(__file__))}/solverCollection/"
    if debug:
        location += "debug/"
    mpcModel.generateSolver(location=location)

if __name__ == "__main__":
    createSolver()
