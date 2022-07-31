import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, acados_ocp_solver 
from typing import List
import matplotlib.pyplot as plt 
import matplotlib.patches as mpatches 
from matplotlib import markers

from dataclasses import dataclass, field
import numpy as np
import casadi as cd 

from plannerbenchmark.generic.planner import Planner, PlannerConfig

from plannerbenchmark.planner.acado_mpc.diff_drive_mpc_model import acados_diff_drive_mpc_model


@dataclass
class AcadosMpcConfig(PlannerConfig):
    # Problem setup 
    # workspace 
    # TODO: Read the workspace limits from the experiment config instead
    ws_x: List[float] = field(default_factory=lambda: [-15.0, 15.0])    # m
    ws_y: List[float] = field(default_factory=lambda: [-16.0, 16.0])      # m
    
    robot_min_vel: float = -1.0
    robot_max_vel: float = 1.0
    robot_max_yawrate: float = 0.26
    robot_min_yawrate: float = -0.26

    # MPC settings
    N: int = 40                  # horizon length
    nx: int = 3                  # state dimension 
    nu: int = 2                  # control dimension 
    nparam: int = 13             # parameter dimension 

    # MPC cost terms weights, can be real time param
    w_pos: float = 2.0
    w_input: float = 0.05
    w_coll: float = 0.06

    # Plot visualization of horizon
    plot: bool = False


# Vector index 
@dataclass
class Index:
    # state vector, x = [px, py, theta]
    x_pos = np.s_[0: 2]     # 0, 1
    x_theta = np.s_[2: 3]   # 2
    # control vector, u = [vel, omega]
    u_vel = np.s_[0: 1]     # 0
    u_omega = np.s_[1: 2]   # 1
    # param vector
    p_robot_pos_start = np.s_[0: 2]     # 0, 1
    p_robot_pos_goal = np.s_[2: 4]      # 2, 3
    p_robot_size = np.s_[4: 6]          # 4, 5
    p_obs_size = np.s_[8: 10]           # 8, 9
    p_obs_pos = np.s_[6: 8]             # 6, 7
    p_mpc_weights = np.s_[10: 13]       # 10, 11, 12
    p_mpc_weights_w_pos = np.s_[0]      # 0 in mpc_weights
    p_mpc_weights_w_input = np.s_[1]    # 1 in mpc_weights
    p_mpc_weights_w_coll = np.s_[2]     # 2 in mpc_weights


class AcadosMpcPlanner(Planner):
    def __init__(self, exp, **kwargs):
        super().__init__(exp, **kwargs)
        self._config = AcadosMpcConfig(**kwargs)

        self._index = Index()

        self._init_problem()

        if self._config.plot:
            self._init_figure()

        self._mpc_feasible = False
        self._mpc_x_plan = np.zeros((self._config.nx, self._config.N))
        self._mpc_u_plan = np.zeros((self._config.nu, self._config.N))

    def _init_problem(self):
        model_ac = acados_diff_drive_mpc_model(self._config, self._index, self._exp)

         # Create an acados ocp object 
        ocp = AcadosOcp()
        ocp.model = model_ac 

        # Set ocp dimensions
        ocp.dims.N = self._config.N           # mandatory 

        # Set cost types
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        # Set initial constraint 
        ocp.constraints.x0 = np.zeros(self._config.nx)    # give dimension here, should be updated when calling solver
        
        # Set state bound 
        ocp.constraints.lbx = np.array([self._config.ws_x[0], self._config.ws_y[0], -np.pi])
        ocp.constraints.ubx = np.array([self._config.ws_x[1], self._config.ws_y[1],  np.pi])
        ocp.constraints.idxbx = np.array(range(self._config.nx))

        # Set control input bound 
        ocp.constraints.lbu = np.array([-self._config.robot_max_vel, -self._config.robot_max_yawrate])
        ocp.constraints.ubu = np.array([ self._config.robot_max_vel,  self._config.robot_max_yawrate])
        ocp.constraints.idxbu = np.array(range(self._config.nu))

        # Set path constraints bound
        ocp.constraints.lh = np.array([0.0])
        ocp.constraints.uh = np.array([100.0])
        # ocp.constraints.lh_e = np.array([0.0])
        # ocp.constraints.uh_e = np.array([100.0])

        # Set parameters 
        ocp.parameter_values = np.zeros((self._config.nparam, ))

        # Solver options
        # horizon
        ocp.solver_options.tf = self._config.N * self._exp.dt()           # mandatory

        # ocp.solver_options.nlp_solver_type = "SQP"
        # ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        # ocp.solver_options.integrator_type = "ERK"
        # ocp.solver_options.sim_method_num_stages = 4
        # ocp.solver_options.sim_method_num_steps = 3

        # integrator option
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 3
        # nlp solver options
        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.nlp_solver_max_iter = 200 
        ocp.solver_options.nlp_solver_tol_eq = 1E-3
        # ocp.solver_options.nlp_solver_tol_ineq = 1E-3
        # ocp.solver_options.nlp_solver_tol_comp = 1E-3
        # ocp.solver_options.nlp_solver_tol_stat = 1E-3
        # hession apself._configoximation 
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        # qp solver options
        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        # ocp.solver_options.qp_solver_cond_N = 5 
        ocp.solver_options.qp_solver_iter_max = 100  
        ocp.solver_options.qp_solver_warm_start = 1 
        # ocp.solver_options.qp_tol = 1E-1
        # code generation options
        ocp.code_export_directory = './diff_drive_mpc_c_generated_code'
        # self._configint options
        ocp.solver_options.print_level = 0
        # ocp.solver_options.levenberg_marquardt = 0.5

        # Generate the solver
        self.acados_ocp_solver = AcadosOcpSolver(ocp, json_file='diff_drive_acados_ocp.json')
        
    def dt(self):
        return self._exp.dt()

    def reset(self):
        pass

    def setObstacles(self, obsts, r_body):
        # NOTE: reinit problem, since the obstacle has changed. 
        # NOTE: only 1 obstacle is supported
        self._init_problem()

    def setSelfCollisionAvoidance(self, r_body):
        pass

    def setJointLimits(self, limits):
        pass

    def setGoal(self, goal):
        # NOTE: reinit problem, since the goal has changed. 
        self._init_problem()

    def concretize(self):
        pass

    def _init_figure(self):
        # Prepare a figure for visualization 
        plt.ion()
        self._fig_main, ax_main = plt.subplots()
        ax_main.grid(visible=True, ls='-.')
        ax_main.set_aspect('equal')
        ax_main.set_xlim(self._config.ws_x)
        ax_main.set_ylim(self._config.ws_y)
        ax_main.set_xlabel('x [m]')
        ax_main.set_ylabel('y [m]')
        # plot obejects
        # obstalce 
        obs_pos_ellipse = mpatches.Ellipse(self._exp.obstacles()[0].position(), 2*self._exp.obstacles()[0].radius(), 2*self._exp.obstacles()[0].radius(), 
                                            angle=0, fc=(1, 0, 0, 0.4), ec=(1, 0, 0, 0.2))
        self._fig_obs_pos = ax_main.add_artist(obs_pos_ellipse)
        # robot current pos
        robot_pos_ellipse = mpatches.Ellipse(self._exp.initState()[0][:2], 2*self._exp.rBody(), 2*self._exp.rBody(), 
                                            angle=np.rad2deg(self._exp.initState()[0][2]), fc=(0, 0, 1, 0.8), ec=(0, 0, 1, 0.8))
        self._fig_robot_pos = ax_main.add_artist(robot_pos_ellipse)
        # robot goal location 
        self._fig_robot_goal = ax_main.plot(self._exp.goal().primeGoal().position()[0], self._exp.goal().primeGoal().position()[1], marker='d', mec='r', mfc='r', ms=10)
        # robot planner path 
        self._fig_robot_mpc_path = ax_main.plot(self._exp.goal().primeGoal().position()[0], self._exp.goal().primeGoal().position()[1], c='c', ls='-', lw=2.0)
        plt.draw()


    def computeAction(self, *args):
        robot_state_current = args[0] # [x, y , yaw]

        print(f"STATE {robot_state_current}")
        self._mpc_x_plan = np.tile(np.array(robot_state_current).reshape((-1, 1)), (1, self._config.N))
        self.acados_ocp_solver.constraints_set(0, 'lbx', np.array(robot_state_current))
        self.acados_ocp_solver.constraints_set(0, 'ubx', np.array(robot_state_current))
        # Set real time paraterms 
        for iStage in range(0, self._config.N):
            param_this_stage = np.zeros((self._config.nparam, ))
            param_this_stage[self._index.p_robot_pos_start] = np.array(robot_state_current[self._index.x_pos])
            param_this_stage[self._index.p_robot_pos_goal] = np.array(self._exp.goal().primeGoal().position())
            param_this_stage[self._index.p_robot_size] = np.array([self._exp.rBody(), self._exp.rBody()])
            param_this_stage[self._index.p_obs_pos] = np.array(self._exp.obstacles()[0].position()) 
            param_this_stage[self._index.p_obs_size] =  np.array([self._exp.obstacles()[0].radius(), self._exp.obstacles()[0].radius()])
            param_this_stage[self._index.p_mpc_weights] = np.array([self._config.w_pos, self._config.w_input, self._config.w_coll])
            if iStage == self._config.N-1:
                param_this_stage[self._index.p_mpc_weights] = np.array([self._config.w_pos, self._config.w_input, self._config.w_coll])
            self.acados_ocp_solver.set(iStage, 'p', param_this_stage)
        # Set initial guess
        if self._mpc_feasible:        # MPC feasible
            x_traj_init = np.concatenate((self._mpc_x_plan[:, 1:], self._mpc_x_plan[:, -1:]), axis=1)
            u_traj_init = np.concatenate((self._mpc_u_plan[:, 1:], self._mpc_u_plan[:, -1:]), axis=1)
        else:                   # MPC infeasible
            x_traj_init = np.tile(np.array(robot_state_current).reshape((-1, 1)), (1, self._config.N))
            u_traj_init = np.zeros((self._config.nu, self._config.N))
        for iStage in range(0, self._config.N):
            self.acados_ocp_solver.set(iStage, 'x', x_traj_init[:, iStage])
            self.acados_ocp_solver.set(iStage, 'u', u_traj_init[:, iStage])
        # Call the solver
        status = self.acados_ocp_solver.solve()

        # NOTE: In case of solver infeasibility, keeps previous action.
        if status != 0:     # infeasible 
            print('acados returned status {} in closed loop iteration.'.format(status))
            self._mpc_feasible = False 
            # use previous velocity
            return list(self._mpc_u_plan[:, 0]) 

        # Obtain solution
        for iStage in range(0, self._config.N):
            self._mpc_x_plan[:, iStage] = self.acados_ocp_solver.get(iStage, 'x')
            self._mpc_u_plan[:, iStage] = self.acados_ocp_solver.get(iStage, 'u')

        self._mpc_feasible = True 
        robot_control_current = list(self._mpc_u_plan[:, 0])

        # Update visualization 
        if self._config.plot:
            self._fig_robot_pos.set_center(robot_state_current[self._index.x_pos])
            self._fig_robot_pos.set_angle(np.rad2deg(robot_state_current[self._index.x_theta]))
            self._fig_robot_mpc_path[0].set_data(self._mpc_x_plan[self._index.x_pos, 1:])
            self._fig_main.canvas.draw()
            self._fig_main.canvas.flush_events()

        print(f"ACTION {robot_control_current}")
        return robot_control_current


