import numpy as np
import casadi as cd 

from acados_template import AcadosModel
from plannerbenchmark.generic.experiment import Experiment 
from plannerbenchmark.planner.acadosMpc.cost_functions import *


def acados_point_mass_model(pr, exp):
    # Create an acados ocp model
    model = AcadosModel()
    model.name = 'point_mass_ocp'

    x = cd.SX.sym('x', 6)           # [px, py, vx, vy]
    x_dot = cd.SX.sym('x_dot', 6)   # [px_dot, py_dot, vx_dot, vy_dot]
    u = cd.SX.sym('u', 3)           # [ax, ay]


    # Helpers
    pos = x[:3]
    vel = x[3:]
    acc = u

    # Dynamics 
    dyn_f_expl = cd.vertcat(vel, u) 
    dyn_f_impl = x_dot - dyn_f_expl 
    
    # Params 
    start = cd.SX.sym('start', 3) # [x, y, theta]
    goal = cd.SX.sym('goal', 2) # [x, y]
    n_obs = len(exp.obstacles())
    obs = cd.SX.sym('obstacles', 4*n_obs) # [x, y, z, radius]*n
    robot_size = cd.SX.sym('r_body', 1)

    # Costs
    cost_goal = l2_normalized(pos[0:2], start[0:2], goal)
    cost_input = input_normalized(u=acc, limit=pr.robot_max_acc)
    cost_coll = sum([custom_collision_potential(pos[0:2], robot_size, obs[i:i+2], obs[i+3]) for i in range(0, n_obs*4, 4)])/n_obs

    # Constraints
    constraints = cd.vertcat(*[collision_constraint(pos[0:2], robot_size, obs[i:i+2], obs[i+3]) for i in range(0, n_obs*4, 4)])

    # stage cost
    cost_stage = pr.w_input*cost_input + pr.w_pos*cost_goal + pr.w_coll*cost_coll
    # terminal cost 
    cost_e = pr.w_pos*cost_goal + pr.w_coll*cost_coll

    # Formulating acados ocp model
    model.x = x 
    model.u = u 
    model.xdot = x_dot 
    model.f_expl_expr = dyn_f_expl 
    model.f_impl_expr = dyn_f_impl 
    model.p = cd.vertcat(start, goal, obs, robot_size)

    model.cost_expr_ext_cost = cost_stage 
    model.cost_expr_ext_cost_e = cost_e 
    model.con_h_expr = constraints

    return model

def point_mass_params(exp: Experiment) -> np.ndarray:
    start = exp.initState()[0]
    goal = exp.goal().primary_goal().position()
    obs = np.array([[*o.position(), o.radius()] for o in exp.obstacles()]).flatten()
    r_body = [exp.rBody()]
    return np.concatenate((start, goal, obs, r_body))
