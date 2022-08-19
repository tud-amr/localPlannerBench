import numpy as np
import casadi as cd 
from forwardkinematics.fksCommon.fk_creator import FkCreator
from plannerbenchmark.planner.acadosMpc.cost_functions import *

from acados_template import AcadosModel 


def acados_n_link_model(pr, exp):
    # Create an acados ocp model
    model = AcadosModel()
    model.name = 'n_link_ocp'

    n = exp.n()
    fk = FkCreator('planarArm', n).fk()

    x = cd.SX.sym('x', n * 2)
    x_dot = cd.SX.sym('x_dot', n * 2)
    u = cd.SX.sym('u', n)          

    # Dynamics 
    dyn_f_expl = cd.vertcat(
        x[n:],
        u
    ) 
    dyn_f_impl = x_dot - dyn_f_expl 

    # Params 
    p = cd.vertcat([])

    # Helpers
    pos = x[:n]
    vel = x[n:]
    acc = u
    
    start_joint_states = np.array(exp.initState()[0][:n])
    start_ee = fk.fk(start_joint_states, n, positionOnly=True)
    goal_ee = np.array(exp.goal().primeGoal().position())
    pos_ee = fk.fk(pos, n, positionOnly=True)

    robot_size = exp.rBody()
    # obs_size = np.array([exp.obstacles()[0].radius(), exp.obstacles()[0].radius()])
    # w_pos = pr.w_pos
    # obs_pos = np.array(exp.obstacles()[0].position())
    # w_input = pr.w_input
    # w_coll = pr.w_coll

    # Costs
    cost_goal = l2_normalized(pos_ee, start_ee, goal_ee)
    cost_input = input_normalized(u=acc, limit=pr.robot_max_acc)
    cost_coll = sum([custom_collision_potential(pos_ee, robot_size, obs.position(), obs.radius()) for obs in exp.obstacles()])/len(exp.obstacles())

    # Constraints
    constraints = [] 
    constraints += [collision_constraint(pos_ee, robot_size, obs.position(), obs.radius()) for obs in exp.obstacles()]
    for i in range(exp.n()):
        if i == 0: continue
        pos_link = fk.fk(pos, i, positionOnly=True)
        constraints += [collision_constraint(pos_link, robot_size, obs.position(), obs.radius()) for obs in exp.obstacles()]
    constraints = cd.vertcat(*constraints)

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
    model.p = p

    model.cost_expr_ext_cost = cost_stage 
    model.cost_expr_ext_cost_e = cost_e 
    model.con_h_expr = constraints

    return model 
