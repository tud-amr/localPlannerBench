from mpscenes.goals.goal_composition import GoalComposition
import dataclasses
import numpy as np
import yaml
import os
import sys
import forcespro
import logging

from dataclasses import dataclass, field
from typing import Dict

from plannerbenchmark.generic.planner import Planner, PlannerConfig
from plannerbenchmark.planner.forcesProMpc.makeSolver import createSolver
from robotmpcs.models.mpcModel import MpcModel
from robotmpcs.models.diff_drive_mpc_model import MpcDiffDriveModel
from robotmpcs.models.mpcModel import MpcConfiguration, RobotConfiguration

path_name = (
    os.path.dirname(os.path.realpath(__file__))
    + "/forcesProMpc/solverCollection/"
)
sys.path.append(path_name)


class EmptyObstacle():
    def position(self):
        return [-100, -100, -100]

    def radius(self):
        return -100

    def dimension(self):
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
        self._config = MpcConfiguration(**kwargs)
        robot_config_dict = {
            'collision_links': self._exp.collision_links(),
            'selfCollision': {
                'pairs': self._exp.selfCollisionPairs()
            },
            'urdf_file': self._exp.urdf_file(),
            'root_link': self._exp.root_link(),
            'end_link': self._exp.ee_links()[0],
            'base_type': self._exp.base_type(),
        }
        self._robot_config = RobotConfiguration(**robot_config_dict)
        dt_str = str(self._config.time_step).replace(".", "")
        debug_folder = ""
        if self._config.debug:
            config = {
                'mpc': dataclasses.asdict(self._config),
                'robot': dataclasses.asdict(self._robot_config),
            }
            if config['robot']['base_type'] == 'holonomic':
                self._mpc_model = MpcModel(initParamMap=True, **config)
            elif config['robot']['base_type'] == 'diffdrive':
                self._mpc_model = MpcDiffDriveModel(initParamMap=True, **config)
            self._mpc_model.setModel()
            debug_folder += "debug/"
        self._solverFile = (
            path_name
            + debug_folder
            + self.robot_identifier()
            + "_n" + str(self._config.n)
            + "_"
            + dt_str
            + "_H"
            + str(self._config.time_horizon)
        ).replace("int", "1nt")
        if not self._config.slack:
            self._solverFile += "_noSlack"
        self.concretize()

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
        self._x0 = np.zeros(shape=(self._config.time_horizon, self._nx + self._nu + self._ns))
        self._xinit = np.zeros(self._nx)
        if self.config.slack:
            self._slack = 0.0
        self._x0[-1, -1] = 0.1
        self._params = np.zeros(shape=(self._npar * self._config.time_horizon), dtype=float)
        for i in range(self._config.time_horizon):
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

    def setObstacles(self, obsts, r_body):
        for i in range(self._config.time_horizon):
            self._params[self._npar * i + self._paramMap["r_body"][0]] = r_body
            for j in range(self._config.number_obstacles):
                if j < len(self._exp.obstacles()):
                    obst = obsts[j]
                else:
                    obst = EmptyObstacle()
                for m_i in range(obst.dimension()):
                    paramsIndexObstX = self._npar * i + self._paramMap['obst'][j * (obst.dimension() + 1) + m_i]
                    self._params[paramsIndexObstX] = obst.position()[m_i]
                paramsIndexObstR = self._npar * i + self._paramMap['obst'][j * (obst.dimension() + 1) + obst.dimension()]
                self._params[paramsIndexObstR] = obst.radius()

    def update_obstacles(self, obst_array: np.ndarray):
        j = -1
        for obst in obst_array:
            j += 1
            obst_position = obst[0]
            obst_velocity = obst[1]
            obst_acceleration = np.zeros_like(obst[0])
            obst_radius = obst[2]
            for i in range(self._config.time_horizon):
                for m_i in range(3):
                    paramsIndexObstX = self._npar * i + self._paramMap['obst'][j * 4 + m_i]
                    predictedPosition =obst_position[m_i] +obst_velocity[m_i] * self._config.time_step * i + 0.5 * (self._config.time_step * i)**2 *obst_acceleration[m_i]
                    self._params[paramsIndexObstX] = predictedPosition
                paramsIndexObstR = self._npar * i + self._paramMap['obst'][j * 4 + 3]
                self._params[paramsIndexObstR] = obst_radius


    def setSelfCollisionAvoidance(self, r_body):
        for i in range(self._config.time_horizon):
            self._params[self._npar * i + self._paramMap["r_body"][0]] = r_body

    def setJointLimits(self, limits):
        for i in range(self._config.time_horizon):
            for j in range(self.config.n):
                self._params[
                    self._npar * i + self._paramMap["lower_limits"][j]
                ] = limits[0][j]
                self._params[
                    self._npar * i + self._paramMap["upper_limits"][j]
                ] = limits[1][j]

    def setGoal(self, goal: GoalComposition):
        if len(goal.sub_goals()) > 1:
            logging.warn("Only single goal supported in mpc")
        primeGoal = goal.primary_goal()
        for i in range(self._config.time_horizon):
            for j, position in enumerate(primeGoal.position()):
                self._params[self._npar * i + self._paramMap["g"][j]] = position

    def update_goal(self, goal: np.ndarray):
        for i in range(self._config.time_horizon):
            for j in range(3):
                self._params[self._npar * i + self._paramMap["g"][j]] = goal[0][0][j] 

    def robot_identifier(self):
        return self._exp.urdf_file().split('.')[0].split('/')[-1]

    def concretize(self):
        self._actionCounter = self.config.interval
        if not os.path.isdir(self._solverFile):
            config = {
                'mpc': dataclasses.asdict(self._config),
                'robot': dataclasses.asdict(self._robot_config),
            }
            createSolver(**config)

        self.load_solver()

    def shiftHorizon(self, output, ob):
        for key in output.keys():
            if self._config.time_horizon < 10:
                stage = int(key[-1])
            elif self._config.time_horizon >= 10 and self._config.time_horizon < 100:
                stage = int(key[-2:])
            elif self._config.time_horizon > 99:
                stage = int(key[-3:])
            if stage == 1:
                continue
            self._x0[stage - 2, 0 : len(output[key])] = output[key]

    def setX0(self, xinit):
        for i in range(self._config.time_horizon):
            self._x0[i][0 : self._nx] = xinit

    def solve(self, **kwargs):
        q = kwargs['joint_state']['position']
        qdot = kwargs['joint_state']['velocity']
        if self._robot_config.base_type == 'diffdrive':
            vel = kwargs['joint_state']['forward_velocity']
            velocity = np.concatenate((vel, qdot[2:]))
            state = np.concatenate((q, qdot, velocity))
        elif self._robot_config.base_type == 'holonomic':
            state = np.concatenate((q, qdot))
        self._xinit = state
        self.update_goal(kwargs['FullSensor']['goals'])
        self.update_obstacles(kwargs['FullSensor']['obstacles'])
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
        output, exitflag, info = self._solver.solve(problem)
        if logging.root.level <= 10:
            inequalities = self._mpc_model.eval_inequalities(self._xinit, self._params)
            not_respected_inequalities = np.where(np.array(inequalities) < 0)
            logging.debug(f"Inequality violations : {not_respected_inequalities}")
            fk = self._mpc_model._fk.fk(
                self._xinit[0:3],
                self._robot_config.root_link, 
                self._robot_config.end_link, 
                positionOnly=True
            )
            obj = self._mpc_model.eval_objective(self._x0[0], self._params)
            logging.debug(obj)

        if exitflag < 0:
            logging.warn(f"MPC solver raised an error flag {exitflag}")
        if  self._config.time_horizon < 10:
            key1 = 'x1'
        elif self._config.time_horizon > 9 and self._config.time_horizon < 100:
            key1 = 'x01'
        elif self._config.time_horizon > 99:
            key1 = 'x001'
        action = output[key1][-self._nu :]
        if self.config.slack:
            self._slack = output[key1][self._nx]
            if self._slack > 1e-3:
                logging.warn(f"Slack variable higher than safe threshold: {self._slack}")
        logging.debug(f"action : {action}")
        logging.debug(f"prediction : {output['x02'][0:self._nx]}")
        self.shiftHorizon(output, state)
        return action, info

    def computeAction(self, **kwargs):
        if self._actionCounter >= self.config.interval:
            self._action, info = self.solve(**kwargs)
            self._actionCounter = 1
        else:
            self._actionCounter += 1
        if self._exp.control_mode() == 'vel':
            return kwargs['joint_state']['velocity'] + self._action * self._exp.dt()
        return self._action

