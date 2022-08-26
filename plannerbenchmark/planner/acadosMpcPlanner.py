import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, acados_ocp_solver 
from typing import List
import matplotlib.pyplot as plt 
import matplotlib.patches as mpatches 
from matplotlib import markers
import logging

from forwardkinematics.fksCommon.fk_creator import FkCreator
from dataclasses import dataclass, field
import numpy as np
import casadi as cd 

from plannerbenchmark.generic.planner import Planner, PlannerConfig

from plannerbenchmark.planner.acadosMpc.createMpcSolver import create_mpc_solver


@dataclass
class AcadosMpcConfig(PlannerConfig):
    # Problem setup 
    # TODO: these should be part of experiment config
    robot_min_acc: float = -1.0
    robot_max_acc: float = 1.0
    robot_min_vel: float = -1.0
    robot_max_vel: float = 1.0

    N: int = 20 # horizon length
    n_obs = 5

    # MPC cost terms weights, can be real time param
    w_pos: float = 1.0
    w_coll: float = 0.10
    w_input: float = 0.01
    w_joints: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    w_vref: float = 0.3


class AcadosMpcPlanner(Planner):
    def __init__(self, exp, **kwargs):
        super().__init__(exp, **kwargs)
        self._config = AcadosMpcConfig(**kwargs)

        self._mpc_feasible = False

        self._init_problem()

    def _init_problem(self):
        # Regenerate solver
        self._acados_ocp_solver, self._extract_params = create_mpc_solver(self._config, self._exp)

        # TODO: add option to load pregenerated solver
        
    def dt(self):
        return self._exp.dt()

    def reset(self):
        pass

    def setObstacles(self, obsts, r_body):
        self._mpc_feasible = False
        pass

    def setSelfCollisionAvoidance(self, r_body):
        pass

    def setJointLimits(self, limits):
        pass

    def setGoal(self, goal):
        self._mpc_feasible = False
        pass

    def concretize(self):
        pass

    def computeAction(self, *args):
        robot_state_current = np.array(args).flatten() # [x, y , vx, vy]
        logging.debug(f"STATE {robot_state_current}")

        # Force solver initial state to be the current robot state
        self._acados_ocp_solver.constraints_set(0, 'lbx', np.array(robot_state_current))
        self._acados_ocp_solver.constraints_set(0, 'ubx', np.array(robot_state_current))

        if not hasattr(self, "_mpc_x_plan"):
            self._mpc_x_plan = np.tile(np.array(robot_state_current).reshape((-1, 1)), (1, self._config.N))

        if not hasattr(self, "_mpc_u_plan"):
            self._mpc_u_plan = np.zeros((self._exp.n(), self._config.N))

        if self._mpc_feasible:
            x_traj_init = np.concatenate((self._mpc_x_plan[:, 1:], self._mpc_x_plan[:, -1:]), axis=1)
            u_traj_init = np.concatenate((self._mpc_u_plan[:, 1:], self._mpc_u_plan[:, -1:]), axis=1)
        else:
            x_traj_init = np.tile(np.array(robot_state_current).reshape((-1, 1)), (1, self._config.N))
            u_traj_init = self._mpc_u_plan = np.zeros((self._exp.n(), self._config.N))

        param_this_stage = self._extract_params(self._exp)

        for iStage in range(0, self._config.N):
            self._acados_ocp_solver.set(iStage, 'x', x_traj_init[:, iStage])
            self._acados_ocp_solver.set(iStage, 'u', u_traj_init[:, iStage])
            self._acados_ocp_solver.set(iStage, 'p', param_this_stage)

        self._acados_ocp_solver.set(iStage+1, 'p', param_this_stage)

        # Call the solver
        status = self._acados_ocp_solver.solve()

        # NOTE: In case of solver infeasibility, keeps previous action.
        if status != 0:     # infeasible 
            # print('acados returned status {} in closed loop iteration.'.format(status))
            self._mpc_feasible = False 
            # use previous action
            return np.array(self._mpc_u_plan[:, 0]) 

        # Obtain solution
        for iStage in range(0, self._config.N):
            self._mpc_x_plan[:, iStage] = self._acados_ocp_solver.get(iStage, 'x')
            self._mpc_u_plan[:, iStage] = self._acados_ocp_solver.get(iStage, 'u')

        self._mpc_feasible = True 
        robot_control_current = list(self._mpc_u_plan[:, 0])

        logging.debug(f"ACTION {robot_control_current}")
        return np.array(robot_control_current)



