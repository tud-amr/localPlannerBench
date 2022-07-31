import numpy as np
import casadi as cd 

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver 

def acados_diff_drive_mpc_model(pr, index, exp):
    """
    pr: 
    index:
    exp:
    """

    # Create an acados ocp model
    model = AcadosModel()
    model.name = 'diff_drive_mpc_ocp'

    x = cd.SX.sym('x', pr.nx)           # [px, py, theta]
    x_dot = cd.SX.sym('x_dot', pr.nx)   # [px_dot, py_dot, theta_dot]
    u = cd.SX.sym('u', pr.nu)           # [vel, omega]

    pos = x[index.x_pos]
    theta = x[index.x_theta]
    vel = u[index.u_vel]
    omega = u[index.u_omega]

    # Params 
    param = cd.SX.sym('params', pr.nparam)
    start = np.array(exp.initState()[0][:2])
    goal = np.array(exp.goal().primeGoal().position())
    robot_size = np.array([exp.rBody(), exp.rBody()])
    obs_pos = np.array(exp.obstacles()[0].position())
    obs_size = np.array([exp.obstacles()[0].radius(), exp.obstacles()[0].radius()])
    w_pos = pr.w_pos
    w_input = pr.w_input
    w_coll = pr.w_coll

    # Dynamics 
    dyn_f_expl = cd.vertcat(
        vel * cd.cos(theta), 
        vel * cd.sin(theta), 
        omega
    ) 
    dyn_f_impl = x_dot - dyn_f_expl 

    # Cost terms 
    def l2_normalized(pos: cd.SX, start: np.ndarray, goal:np.ndarray) -> cd.SX:
        start_to_goal = (goal - start).T @ (goal - start)
        start_to_goal = cd.fmax(start_to_goal, 1) # clip normalization
        pos_to_goal = (goal - pos).T @ (goal - pos)
        return pos_to_goal / start_to_goal 

    cost_goal = l2_normalized(pos, start, goal)**0.7

    u_normalized = cd.vertcat(vel/pr.robot_max_vel, omega/pr.robot_max_yawrate)
    cost_input = u_normalized.T @ u_normalized

    def custom_collision_potential(pos: cd.SX, robot_size: np.ndarray, obs_pos: np.ndarray, obs_size: np.ndarray) -> cd.SX:
        ell_axis = robot_size + 1.6*obs_size
        d_vec = obs_pos - pos 
        d_vec_ell = d_vec / ell_axis        # elementwise division 
        d = d_vec_ell.T @ d_vec_ell
        # cost = 1.0 / (1.0 + cd.exp(10.0*(d - 1.6)))
        cost = 1.0 / (1.0 + cd.exp(10.0*(d)))

        return cost 

    cost_coll = custom_collision_potential(pos, robot_size, obs_pos, obs_size)

    # Path constraints
    safety_factor = 1.05
    pos_to_obs = ((obs_pos - pos).T @ (obs_pos - pos))**0.5
    constraint_coll = pos_to_obs - (robot_size[0] + obs_size[0])*safety_factor

    # stage cost
    cost_stage = w_input*cost_input + w_pos*cost_goal + w_coll*cost_coll
    # terminal cost 
    cost_e = w_pos*cost_goal + w_coll*cost_coll

    # Formulating acados ocp model
    model.x = x 
    model.u = u 
    model.xdot = x_dot 
    model.p = param 
    model.f_expl_expr = dyn_f_expl 
    model.f_impl_expr = dyn_f_impl 

    model.cost_expr_ext_cost = cost_stage 
    model.cost_expr_ext_cost_e = cost_e 
    model.con_h_expr = constraint_coll

    return model 
