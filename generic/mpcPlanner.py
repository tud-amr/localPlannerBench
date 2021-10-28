import numpy as np
import yaml
import os
import sys
import forcespro


from fabricsExperiments.generic.abstractPlanner import AbstractPlanner
from fabricsExperiments.generic.mpc.parameterMap import getParameterMap
from fabricsExperiments.generic.mpc.createMPCSolver import eval_ineq, eval_obj
import fabricsExperiments

from obstacle import Obstacle

path_name = (
    os.path.dirname(os.path.realpath(fabricsExperiments.__file__))
    + "/generic/mpc/solverCollection/"
)
sys.path.append(path_name)


class MPCPlanner(AbstractPlanner):
    def __init__(self, exp, setupFile):
        required_keys = ["type", "n", "obst", "weights", "interval", "H", "dt"]
        super().__init__(exp, setupFile, required_keys)
        self._paramMap, self._npar, self._nx, self._nu, self._ns = getParameterMap(
            self.n(), self.m(), self.nbObstacles()
        )
        dt_str = str(self.dt()).replace(".", "")
        self._solverFile = (
            path_name
            + self._exp.robotType()
            + "/solver_n"
            + str(self.n())
            + "_"
            + dt_str
            + "_H"
            + str(self.H())
        )
        try:
            self._solver = forcespro.nlp.Solver.from_directory(self._solverFile)
        except Exception as e:
            raise e
            __import__('pdb').set_trace()

    def reset(self):
        print("RESETTING PLANNER")
        self._x0 = np.zeros(shape=(self.H(), self._nx + self._nu + self._ns))
        self._xinit = np.zeros(self._nx)
        self._slack = 0.0
        self._x0[-1, -1] = 0.1
        self._params = np.zeros(shape=(self._npar * self.H()), dtype=float)
        for i in range(self.H()):
            self._params[
                [self._npar * i + val for val in self._paramMap["w"]]
            ] = self.weights()["wx"]
            self._params[
                [self._npar * i + val for val in self._paramMap["wvel"]]
            ] = self.weights()["wvel"]
            self._params[
                [self._npar * i + val for val in self._paramMap["wu"]]
            ] = self.weights()["wu"]
            self._params[
                [self._npar * i + val for val in self._paramMap["ws"]]
            ] = self.weights()["ws"]

    def m(self):
        return 2

    def interval(self):
        return self._setup["interval"]

    def n(self):
        return self._setup["n"]

    def H(self):
        return self._setup["H"]

    def dt(self):
        return self._setup["dt"]

    def weights(self):
        return self._setup["weights"]

    def nbObstacles(self):
        return self._setup["obst"]["nbObst"]

    def setObstacles(self, obsts, r_body):
        for i in range(self.H()):
            self._params[self._npar * i + self._paramMap["r_body"][0]] = r_body
            for j in range(self.nbObstacles()):
                if j < len(obsts):
                    obst = obsts[j]
                else:
                    obst = Obstacle(np.array([-10.0, 10.0]), -1.0)
                self._params[
                    self._npar * i + self._paramMap["obst"][j * 3 + 0]
                ] = obst.x()[0]
                self._params[
                    self._npar * i + self._paramMap["obst"][j * 3 + 1]
                ] = obst.x()[1]
                self._params[
                    self._npar * i + self._paramMap["obst"][j * 3 + 2]
                ] = obst.r()

    def setSelfCollisionAvoidance(self, r_body):
        for i in range(self.H()):
            self._params[self._npar * i + self._paramMap["r_body"][0]] = r_body

    def setJointLimits(self, limits):
        for i in range(self.H()):
            for j in range(self.n()):
                self._params[
                    self._npar * i + self._paramMap["lower_limits"][j]
                ] = limits[0][j]
                self._params[
                    self._npar * i + self._paramMap["upper_limits"][j]
                ] = limits[1][j]

    def setGoal(self, goal):
        if len(goal.subGoals()) > 1:
            print("WARNING: Only single goal supported in mpc")
        primeGoal = goal.primeGoal()
        for i in range(self.H()):
            for j in range(self.m()):
                self._params[self._npar * i + self._paramMap["g"][j]] = primeGoal[j]

    def concretize(self):
        pass

    def shiftHorizon(self, output, ob):
        for key in output.keys():
            stage = int(key[-2:])
            if stage == 1:
                continue
            self._x0[stage - 2, 0 : len(output[key])] = output[key]

    def setX0(self, xinit):
        for i in range(self.H()):
            self._x0[i][0 : self._nx] = xinit

    def solve(self, ob):
        # print("Observation : " , ob[0:self._nx])
        self._xinit = ob[0 : self._nx]
        action = np.zeros(self._nu)
        problem = {}
        # problem["ToleranceStationarity"] = 1e-7
        # problem["ToleranceEqualities"] = 1e-7
        # problem["ToleranceInequalities"] = 1e-7
        # problem["SolverTimeout"] = 0.0001
        # problem["ToleranceComplementarity"] = 1e-5
        problem["xinit"] = self._xinit
        # problem["x0"] = self._x0.flatten()[:-self._nu]
        self._x0[0][0 : self._nx] = self._xinit
        self.setX0(self._xinit)
        problem["x0"] = self._x0.flatten()[:]
        problem["all_parameters"] = self._params
        # debug
        debug = False
        if debug:
            nbPar = int(len(self._params)/self.H())
            z = np.concatenate((self._xinit, np.array([self._slack, 0, 0])))
            p = self._params[0:nbPar]
            J = eval_obj(z, p)
            ineq = eval_ineq(z, p)
            print("ineq : ", ineq)
            for i in range(self.H()):
                z = self._x0[i]
                ineq = eval_ineq(z, p)
            #print("J : ", J)
            #print('z : ', z)
            #print('xinit : ', self._xinit)
        output, exitflag, info = self._solver.solve(problem)
        if exitflag < 0:
            print(exitflag)
        action = output["x01"][-self._nu :]
        self._slack = output["x01"][self._nx]
        if self._slack > 1e-3:
            print("slack : ", self._slack)
        # print('action : ', action)
        # print("prediction : ", output["x02"][0:self._nx])
        self.shiftHorizon(output, ob)
        return action, info

    def concretize(self):
        self._actionCounter = self.interval()

    def computeAction(self, q, qdot):
        ob = np.concatenate((q, qdot))
        if self._actionCounter >= self.interval():
            self._action, info = self.solve(ob)
            self._actionCounter = 1
        else:
            self._actionCounter += 1
        return self._action


if __name__ == "__main__":
    test_setup = "testSetupFiles/mpcPlanar.yaml"
    myMPCPlanner = MPCPlanner(None, test_setup)
    myMPCPlanner.reset()
