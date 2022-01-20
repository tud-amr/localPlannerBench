from robotmpcs.models.pointRobotMpcModel import PointRobotMpcModel
from robotmpcs.models.planarArmMpcModel import PlanarArmMpcModel
#from robotmpcs.boxerMpcModel import BoxerMpcModel
from robotmpcs.models.pandaMpcModel import PandaMpcModel


def createSolver():
    robotType = "pointRobot"
    debug = False
    slack = True
    dt = 0.5
    N = 10
    if robotType == 'planarArm':
        n = 2
        mpcModel = PlanarArmMpcModel(2, N, n)
    elif robotType == 'pointRobot':
        n = 2
        mpcModel = PointRobotMpcModel(2, N)
    elif robotType == 'panda':
        n = 7
        mpcModel = PandaMpcModel(3, N)
    else:
        print(f"Solver for robot type {robotType} not available yet")
        
    if slack:
        mpcModel.setSlack()
    mpcModel.setDt(dt)
    mpcModel.setObstacles(5, 2, inCostFunction=False)
    mpcModel.setModel()
    mpcModel.setCodeoptions(debug=debug)
    location = "./solverCollection/"
    if debug:
        location += "debug/"
    mpcModel.generateSolver(location=location)

if __name__ == "__main__":
    createSolver()
