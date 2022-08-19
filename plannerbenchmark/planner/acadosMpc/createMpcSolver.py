import numpy as np
from plannerbenchmark.planner.acadosMpc.pointMassModel import acados_point_mass_model
from plannerbenchmark.planner.acadosMpc.nLinkModel import acados_n_link_model
from acados_template import AcadosOcp, AcadosOcpSolver

import os

def create_mpc_solver(pr, exp):

    # TODO: support more experiment envs
    if exp.envName() == "point-robot-acc-v0":
        model_ac = acados_point_mass_model(pr, exp)
    elif exp.envName() == "nLink-reacher-acc-v0":
        model_ac = acados_n_link_model(pr, exp)

     # Create an acados ocp object 
    ocp = AcadosOcp()
    ocp.model = model_ac 

    # Set ocp dimensions
    ocp.dims.N = pr.N           # mandatory 

    # Set cost types
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'

    # Set initial constraint 
    ocp.constraints.x0 = np.zeros(exp.n()*2)
    
    # Set state bound 
    ocp.constraints.lbx = np.array([[pr.ws_x[0]]*exp.n(), [pr.robot_min_vel]*exp.n()]).flatten()
    ocp.constraints.ubx = np.array([[pr.ws_x[1]]*exp.n(), [pr.robot_max_vel]*exp.n()]).flatten()
    ocp.constraints.idxbx = np.array(range(exp.n()*2))

    # Set control input bound 
    ocp.constraints.lbu = np.array([pr.robot_min_acc]*exp.n())
    ocp.constraints.ubu = np.array([pr.robot_max_acc]*exp.n())
    ocp.constraints.idxbu = np.array(range(exp.n()))

    # Set path constraints bound
    ocp.constraints.lh = np.zeros(len(exp.obstacles()))
    ocp.constraints.uh = np.ones(len(exp.obstacles()))*100
    # Slack for constraints
    # ocp.constraints.idxsh = np.array([0])
    ns = ocp.model.con_h_expr.shape[0]
    nsh = ns
    ocp.cost.zl = 100 * np.ones((ns,))
    ocp.cost.zu = 100 * np.ones((ns,))
    ocp.cost.Zl = 1 * np.ones((ns,))
    ocp.cost.Zu = 1 * np.ones((ns,))
    ocp.constraints.lsh = np.zeros(nsh)
    ocp.constraints.ush = np.zeros(nsh)
    ocp.constraints.idxsh = np.array(range(nsh))

    # horizon
    ocp.solver_options.tf = pr.N * exp.dt()

    # Solver options
    # integrator option
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    # nlp solver options
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.nlp_solver_max_iter = 400 
    ocp.solver_options.nlp_solver_tol_eq = 1E-3
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    # qp solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_iter_max = 200  
    ocp.solver_options.qp_solver_warm_start = 1 
    # code generation options
    ocp.code_export_directory = f"{os.path.dirname(os.path.abspath(__file__))}/point_mass_mpc_c_generated_code"
    ocp.solver_options.print_level = 0

    # Generate the solver
    return AcadosOcpSolver(acados_ocp=ocp, json_file=f"{os.path.dirname(os.path.abspath(__file__))}/point_mass_acados_ocp.json")

