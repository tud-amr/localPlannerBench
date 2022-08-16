import numpy as np
from plannerbenchmark.planner.acadosMpc.pointMassModel import acados_point_mass_model
from acados_template import AcadosOcp, AcadosOcpSolver

import os

def create_mpc_solver(pr, exp):

    # TODO: support more experiment envs
    assert exp.envName() == "point-robot-acc-v0"
    model_ac = acados_point_mass_model(pr, exp)

     # Create an acados ocp object 
    ocp = AcadosOcp()
    ocp.model = model_ac 

    # Set ocp dimensions
    ocp.dims.N = pr.N           # mandatory 

    # Set cost types
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'

    # Set initial constraint 
    ocp.constraints.x0 = np.zeros(pr.nx)
    
    # Set state bound 
    ocp.constraints.lbx = np.array([pr.ws_x[0], pr.ws_y[0], pr.robot_min_vel, pr.robot_min_vel])
    ocp.constraints.ubx = np.array([pr.ws_x[1], pr.ws_y[1], pr.robot_max_vel, pr.robot_max_vel])
    ocp.constraints.idxbx = np.array(range(pr.nx))

    # Set control input bound 
    ocp.constraints.lbu = np.array([pr.robot_min_acc, pr.robot_min_acc])
    ocp.constraints.ubu = np.array([pr.robot_max_acc, pr.robot_max_acc])
    ocp.constraints.idxbu = np.array(range(pr.nu))

    # Set path constraints bound
    ocp.constraints.lh = np.array([0.2])
    ocp.constraints.uh = np.array([100.0])
    # horizon
    ocp.solver_options.tf = pr.N * exp.dt()

    # Solver options
    # integrator option
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    # nlp solver options
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.nlp_solver_max_iter = 200 
    ocp.solver_options.nlp_solver_tol_eq = 1E-3
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    # qp solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_iter_max = 100  
    ocp.solver_options.qp_solver_warm_start = 1 
    # code generation options
    ocp.code_export_directory = f"{os.path.dirname(os.path.abspath(__file__))}/point_mass_mpc_c_generated_code"
    ocp.solver_options.print_level = 0

    # Generate the solver
    return AcadosOcpSolver(acados_ocp=ocp, json_file=f"{os.path.dirname(os.path.abspath(__file__))}/point_mass_acados_ocp.json")

