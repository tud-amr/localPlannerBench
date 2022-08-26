import numpy as np
import casadi as cd 
from forwardkinematics.fksCommon.fk_creator import FkCreator
from plannerbenchmark.generic.experiment import Experiment
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

    # Helpers
    pos = x[:n]
    vel = x[n:]
    acc = u
    

    # Params 
    start_joint_states = cd.SX.sym('start', n) # [x, y]
    start_ee = fk.fk(start_joint_states, n, positionOnly=True)
    goal_ee = cd.SX.sym('goal', 2) # [x, y]
    pos_ee = fk.fk(pos, n, positionOnly=True)
    n_obs = len(exp.obstacles())
    obs = cd.SX.sym('obstacles', 3*n_obs) # [x, y, radius]*n
    robot_size = cd.SX.sym('r_body', 1) 

    # Costs
    cost_goal = l2_normalized(pos_ee, start_ee, goal_ee)
    cost_input = input_normalized(u=acc, limit=pr.robot_max_acc)
    cost_coll = sum([custom_collision_potential(pos_ee, robot_size, obs[i:i+2], obs[i+2]) for i in range(0, n_obs*3, 3)])/n_obs

    # Constraints
    constraints = [] 
    for i in range(exp.n()+1):
        if i == 0: continue
        pos_link = fk.fk(pos, i, positionOnly=True)
        constraints += [collision_constraint(pos_link, robot_size, obs[i:i+2], obs[i+2]) for i in range(0, n_obs*3, 3)]
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
    model.p = cd.vertcat(start_joint_states, goal_ee, obs, robot_size)

    model.cost_expr_ext_cost = cost_stage 
    model.cost_expr_ext_cost_e = cost_e 
    model.con_h_expr = constraints



    return model 

def n_link_params(exp: Experiment) -> np.ndarray:
    start_joint_states = np.array(exp.initState()[0])
    goal_ee = np.array(exp.goal().primeGoal().position())
    obs = np.array([[*o.position(), o.radius()] for o in exp.obstacles()]).flatten()
    r_body = [exp.rBody()]
    return np.concatenate((start_joint_states, goal_ee, obs, r_body))
