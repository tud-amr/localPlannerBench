import numpy as np
import yaml
import os
import sys
import forcespro
import logging

from dataclasses import dataclass, field
from typing import Dict

from plannerbenchmark.generic.planner import Planner, PlannerConfig

path_name = (
    os.path.dirname(os.path.realpath(__file__))
    + "/mpc/solverCollection/"
)
sys.path.append(path_name)


class EmptyObstacle():
    def position(self):
        return [-100, -100, -100]

    def radius(self):
        return -100

    def dim(self):
        return 3

@dataclass
class ForcesProMpcConfig(PlannerConfig):
    H: int = 10
    dt: float = 0.5
    slack: bool = False
    dynamic: bool = False
    obst: Dict[str, int] = field(default_factory = lambda : ({'nbObst': 5}))
    weights: Dict[str, float] = field(default_factory = lambda : ({'ws': 1e7, 'wu': 1, 'wvel': 1, 'wx': 1}))



class ForcesProMpcPlanner(Planner):
    def __init__(self, exp, **kwargs):
        super().__init__(exp, **kwargs)
        self._config = ForcesProMpcConfig(**kwargs)
        """
        self._paramMap, self._npar, self._nx, self._nu, self._ns = getParameterMap(
            self.config.n, self.m(), self.nbObstacles(), self.m(), self.config.slack
        )
        """
        dt_str = str(self.config.dt).replace(".", "")
        debugFolder = ""
        self._solverFile = (
            path_name
            + self._exp.robotType()
            + "_n" + str(self.config.n)
            + "_"
            + dt_str
            + "_H"
            + str(self.config.H)
        ).replace("int", "1nt")
        if not self.config.slack:
            self._solverFile += "_noSlack"
        self.load_solver()

    def load_solver(self):
        logging.info(f"Loading solver {self._solverFile}")
        try:
            with open(self._solverFile + "/paramMap.yaml", "r") as stream:
                self._paramMap = yaml.safe_load(stream)
            with open(self._solverFile + "/properties.yaml", "r") as stream:
                self._properties = yaml.safe_load(stream)
            self._solver = forcespro.nlp.Solver.from_directory(self._solverFile)
        except FileNotFoundError as file_not_found_error:
            logging.error("Solver has not been generated. Consider creating it using `makeSolver.py`")
            raise file_not_found_error
        except Exception as e:
            logging.error("Failed to load solver")
            raise e
        self._npar = self._properties['npar']
        self._nx = self._properties['nx']
        self._nu = self._properties['nu']
        self._ns = self._properties['ns']

    def reset(self):
        logging.info("Resetting mpc planner.")
        self._x0 = np.zeros(shape=(self.config.H, self._nx + self._nu + self._ns))
        self._xinit = np.zeros(self._nx)
        if self.config.slack:
            self._slack = 0.0
        self._x0[-1, -1] = 0.1
        self._params = np.zeros(shape=(self._npar * self.config.H), dtype=float)
        for i in range(self.config.H):
            self._params[
                [self._npar * i + val for val in self._paramMap["w"]]
            ] = self.config.weights["wx"]
            self._params[
                [self._npar * i + val for val in self._paramMap["wvel"]]
            ] = self.config.weights["wvel"]
            self._params[
                [self._npar * i + val for val in self._paramMap["wu"]]
            ] = self.config.weights["wu"]
            if self.config.slack:
                self._params[
                    [self._npar * i + val for val in self._paramMap["ws"]]
                ] = self.config.weights["ws"]
            if 'wobst' in self.config.weights:
                self._params[
                    [self._npar * i + val for val in self._paramMap["wobst"]]
                ] = self.config.weights["wobst"]

    def m(self):
        if self._exp.robotType() == 'panda':
            return 3
        else:
            return 2

    def setObstacles(self, obsts, r_body):
        self._r = obsts[0].radius()
        for i in range(self.config.H):
            self._params[self._npar * i + self._paramMap["r_body"][0]] = r_body
            for j in range(self.config.obst['nbObst']):
                if j < len(obsts):
                    obst = obsts[j]
                else:
                    obst = EmptyObstacle()
                for m_i in range(obst.dim()):
                    paramsIndexObstX = self._npar * i + self._paramMap['obst'][j * (self.m() + 1) + m_i]
                    self._params[paramsIndexObstX] = obst.position()[m_i]
                paramsIndexObstR = self._npar * i + self._paramMap['obst'][j * (self.m() + 1) + self.m()]
                self._params[paramsIndexObstR] = obst.radius()

    def updateDynamicObstacles(self, obstArray):
        nbDynamicObsts = int(obstArray.size / 3 / self.m())
        for j in range(self.config.nbObst):
            if j < nbDynamicObsts:
                obstPos = obstArray[:self.m()]
                obstVel = obstArray[self.m():2*self.m()]
                obstAcc = obstArray[2*self.m():3*self.m()]
            else:
                obstPos = np.ones(self.m()) * -100
                obstVel = np.zeros(self.m())
                obstAcc = np.zeros(self.m())
            for i in range(self.config.H):
                for m_i in range(self.m()):
                    paramsIndexObstX = self._npar * i + self._paramMap['obst'][j * (self.m() + 1) + m_i]
                    predictedPosition = obstPos[m_i] + obstVel[m_i] * self.dt() * i + 0.5 * (self.dt() * i)**2 * obstAcc[m_i]
                    self._params[paramsIndexObstX] = predictedPosition
                paramsIndexObstR = self._npar * i + self._paramMap['obst'][j * (self.m() + 1) + self.m()]
                self._params[paramsIndexObstR] = self._r

    def setSelfCollisionAvoidance(self, r_body):
        for i in range(self.config.H):
            self._params[self._npar * i + self._paramMap["r_body"][0]] = r_body

    def setJointLimits(self, limits):
        for i in range(self.config.H):
            for j in range(self.config.n):
                self._params[
                    self._npar * i + self._paramMap["lower_limits"][j]
                ] = limits[0][j]
                self._params[
                    self._npar * i + self._paramMap["upper_limits"][j]
                ] = limits[1][j]

    def setGoal(self, goal):
        if len(goal.subGoals()) > 1:
            logging.warn("Only single goal supported in mpc")
        primeGoal = goal.primeGoal()
        for i in range(self.config.H):
            for j in range(self.m()):
                self._params[self._npar * i + self._paramMap["g"][j]] = primeGoal.position()[j]

    def concretize(self):
        pass

    def shiftHorizon(self, output, ob):
        for key in output.keys():
            if self.config.H < 10:
                stage = int(key[-1])
            elif self.config.H >= 10 and self.config.H < 100:
                stage = int(key[-2:])
            elif self.config.H > 99:
                stage = int(key[-3:])
            if stage == 1:
                continue
            self._x0[stage - 2, 0 : len(output[key])] = output[key]

    def setX0(self, xinit):
        for i in range(self.config.H):
            self._x0[i][0 : self._nx] = xinit

    def solve(self, ob):
        self._xinit = ob[0 : self._nx]
        if ob.size > self._nx:
            self.updateDynamicObstacles(ob[self._nx:])
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
            nbPar = int(len(self._params)/self.config.H)
            if self.config.slack:
                z = np.concatenate((self._xinit, np.array([self._slack])))
            else:
                z = self._xinit
            p = self._params[0:nbPar]
            #J = eval_obj(z, p)
            ineq = eval_ineq(z, p)
            #print("ineq : ", ineq)
            # __import__('pdb').set_trace()
            """
            for i in range(self.config.H):
                z = self._x0[i]
                ineq = eval_ineq(z, p)
            """
            #print("J : ", J)
            #print('z : ', z)
            #print('xinit : ', self._xinit)
        output, exitflag, info = self._solver.solve(problem)
        if exitflag < 0:
            logging.warn(f"MPC solver raised an error flag {exitflag}")
        if  self.config.H < 10:
            key1 = 'x1'
        elif self.config.H > 9 and self.config.H < 100:
            key1 = 'x01'
        elif self.config.H > 99:
            key1 = 'x001'
        action = output[key1][-self._nu :]
        if self.config.slack:
            self._slack = output[key1][self._nx]
            if self._slack > 1e-3:
                logging.warn(f"Slack variable higher than safe threshold: {self._slack}")
        logging.debug(f"action : {action}")
        logging.debug(f"prediction : {output['x02'][0:self._nx]}")
        self.shiftHorizon(output, ob)
        return action, info

    def concretize(self):
        self._actionCounter = self.config.interval

    def computeAction(self, *args):
        ob = np.concatenate(args)
        if self._actionCounter >= self.config.interval:
            self._action, info = self.solve(ob)
            self._actionCounter = 1
        else:
            self._actionCounter += 1
        return self._action

