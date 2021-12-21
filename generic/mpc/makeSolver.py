from robot_mpcs.pointRobotMpcModel import PointRobotMpcModel
from robot_mpcs.planarArmMpcModel import PlanarArmMpcModel
from robot_mpcs.boxerMpcModel import BoxerMpcModel

def createSolver():
    robotType = "boxer"
    debug = False
    slack = False
    dt = 0.5
    N = 10
    if robotType == 'planarArm':
        n = 2
        mpcModel = PlanarArmMpcModel(2, N, n)
    elif robotType == 'pointMass':
        n = 2
        mpcModel = PointRobotMpcModel(2, N)
    elif robotType == 'boxer':
        nu = 2
        nx = 3
        mpcModel = BoxerMpcModel(nu, nx, N)
    solverName = 'solver_n' + str(n) + '_' + str(dt).replace('.', '') + '_H' + str(N)
    if slack:
        mpcModel.setSlack()
    else:
        solverName += "_noSlack"
    mpcModel.setDt(dt)
    mpcModel.setObstacles(5, 2, inCostFunction=True)
    mpcModel.setModel()
    mpcModel.setCodeoptions(solverName, debug=debug)
    location = "./solverCollection/" + robotType + "/"
    if debug:
        location += "debug/"
    mpcModel.generateSolver(location=location)

if __name__ == "__main__":
    createSolver()
