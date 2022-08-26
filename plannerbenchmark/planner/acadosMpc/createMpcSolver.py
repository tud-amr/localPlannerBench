import numpy as np
from plannerbenchmark.planner.acadosMpc.models.nLinkModel import acados_n_link_model, n_link_params
from plannerbenchmark.planner.acadosMpc.models.pandaArmModel import acados_panda_arm_model, panda_arm_params
from plannerbenchmark.planner.acadosMpc.models.pointMassModel import acados_point_mass_model, point_mass_params
from acados_template import AcadosOcp, AcadosOcpSolver

import os

def create_mpc_solver(pr, exp):

    # TODO: support more experiment envs
    if exp.envName() == "point-robot-acc-v0":
        model_ac = acados_point_mass_model(pr, exp)
        extract_params = point_mass_params
    elif exp.envName() == "nLink-reacher-acc-v0":
        model_ac = acados_n_link_model(pr, exp)
        extract_params = n_link_params
    elif exp.envName() == "panda-reacher-acc-v0":
        model_ac = acados_panda_arm_model(pr, exp)
        extract_params = panda_arm_params

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
    nx = model_ac.x.size()[0]
    ocp.constraints.lbx = np.array([exp.limits()[0], [pr.robot_min_vel]*exp.n()]).flatten()
    ocp.constraints.ubx = np.array([exp.limits()[1], [pr.robot_max_vel]*exp.n()]).flatten()
        
    ocp.constraints.idxbx = np.array(range(exp.n()*2))

    # Set control input bound 
    ocp.constraints.lbu = np.array([pr.robot_min_acc]*exp.n())
    ocp.constraints.ubu = np.array([pr.robot_max_acc]*exp.n())
    ocp.constraints.idxbu = np.array(range(exp.n()))

    # Set path constraints bound
    nc = ocp.model.con_h_expr.shape[0]
    ocp.constraints.lh = np.zeros(nc)
    ocp.constraints.uh = np.ones(nc)*100

    # Slack for constraints
    ns = nc + nx
    ocp.constraints.idxsh = np.array(range(nc))

    # Slack for state bounds
    ocp.constraints.idxsbx = np.array(range(nx))

    ocp.cost.zl = 1e2 * np.ones((ns,))
    ocp.cost.zu = 1e2 * np.ones((ns,))
    ocp.cost.Zl = 1e0 * np.ones((ns,))
    ocp.cost.Zu = 1e0 * np.ones((ns,))

    # ocp.constraints.idxsbx_e = np.array(range(nx))
    # # ocp.constraints.idxsh_e = np.array([0])
    # ocp.cost.zl_e = 1e2 * np.ones(nx) 
    # ocp.cost.zu_e = 1e2 * np.ones(nx) 
    # ocp.cost.Zu_e = 1.0 * np.ones(nx) 
    # ocp.cost.Zl_e = 1.0 * np.ones(nx) 

    ocp.parameter_values = np.zeros(model_ac.p.size()[0])

    # horizon
    ocp.solver_options.tf = pr.N * exp.dt()
    ocp.solver_options.tol = 1e-3

    # Solver options
    # integrator option
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    # nlp solver options
    ocp.solver_options.nlp_solver_type = 'SQP'
    ocp.solver_options.nlp_solver_max_iter = 400 
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    # qp solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_iter_max = 100  

    # code generation options
    ocp.code_export_directory = f"{os.path.dirname(os.path.abspath(__file__))}/point_mass_mpc_c_generated_code"
    ocp.solver_options.print_level = 0

    # Generate the solver
    return AcadosOcpSolver(acados_ocp=ocp, json_file=f"{os.path.dirname(os.path.abspath(__file__))}/point_mass_acados_ocp.json"), extract_params

