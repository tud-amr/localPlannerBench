import numpy as np
import casadi as cd 

from acados_template import AcadosModel 
from plannerbenchmark.planner.acadosMpc.cost_functions import *


def acados_point_mass_model(pr, exp):
    # Create an acados ocp model
    model = AcadosModel()
    model.name = 'point_mass_ocp'

    x = cd.SX.sym('x', 4)           # [px, py, vx, vy]
    x_dot = cd.SX.sym('x_dot', 4)   # [px_dot, py_dot, vx_dot, vy_dot]
    u = cd.SX.sym('u', 2)           # [ax, ay]

    # Dynamics 
    dyn_f_expl = cd.vertcat(x[2:], u) 
    dyn_f_impl = x_dot - dyn_f_expl 

    # Helpers
    pos = x[:2]
    vel = x[2:]
    acc = u
    
    # Params 
    start = np.array(exp.initState()[0][:2])
    goal = np.array(exp.goal().primeGoal().position())
    robot_size = exp.rBody()

    # Costs
    cost_goal = l2_normalized(pos, start, goal)
    cost_input = input_normalized(u=acc, limit=pr.robot_max_acc)
    cost_coll = sum([custom_collision_potential(pos, robot_size, obs.position(), obs.radius()) for obs in exp.obstacles()])/len(exp.obstacles())

    # Constraints
    constraints = cd.vertcat(*[collision_constraint(pos, robot_size, obs.position(), obs.radius()) for obs in exp.obstacles()])

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

    model.cost_expr_ext_cost = cost_stage 
    model.cost_expr_ext_cost_e = cost_e 
    model.con_h_expr = constraints

    return model 
