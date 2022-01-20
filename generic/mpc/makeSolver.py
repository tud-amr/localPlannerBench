from robotmpcs.pointRobotMpcModel import PointRobotMpcModel
from robotmpcs.planarArmMpcModel import PlanarArmMpcModel
#from robotmpcs.boxerMpcModel import BoxerMpcModel
from robotmpcs.pandaMpcModel import PandaMpcModel

def createSolver():
    robotType = "panda"
    debug = False
    slack = True
    dt = 0.1
    N = 30
    if robotType == 'planarArm':
        n = 2
        mpcModel = PlanarArmMpcModel(2, N, n)
    elif robotType == 'pointMass':
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
    mpcModel.setObstacles(5, 3, inCostFunction=False)
    mpcModel.setModel()
    mpcModel.setCodeoptions(solverName, debug=debug)
    location = "./solverCollection/" + robotType + "/"
    if debug:
        location += "debug/"
    mpcModel.generateSolver(location=location)

if __name__ == "__main__":
    createSolver()
