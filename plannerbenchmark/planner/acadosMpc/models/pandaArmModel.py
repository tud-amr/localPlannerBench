import numpy as np
import casadi as cd 
from forwardkinematics.urdfFks.pandaFk import PandaFk
from plannerbenchmark.generic.experiment import Experiment
from plannerbenchmark.planner.acadosMpc.cost_functions import *

from acados_template import AcadosModel 


def acados_panda_arm_model(pr, exp):
    # Create an acados ocp model
    model = AcadosModel()
    model.name = 'panda_arm_ocp'

    n = exp.n()
    fk = PandaFk()

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
    goal_ee = cd.SX.sym('goal', 3) # [x, y, z]
    pos_ee = fk.fk(pos, n, positionOnly=True)
    n_obs = len(exp.obstacles())
    obs = cd.SX.sym('obstacles', 4*n_obs) # [x, y, z, radius]*n
    robot_size = cd.SX.sym('r_body', 1) 

    # Costs
    cost_goal = l2_normalized(pos_ee, start_ee, goal_ee)
    cost_input = input_normalized(u=acc, limit=pr.robot_max_acc)
    cost_coll = sum([custom_collision_potential(pos_ee, robot_size, obs[i:i+3], obs[i+3]) for i in range(0, n_obs*4, 4)])/n_obs
    center_joints = (exp.limits()[0] + exp.limits()[1]) / 2.
    cost_joints = input_normalized(pos - center_joints, 2)
    cost_vref = input_normalized(vel, pr.robot_max_vel)

    # Constraints
    constraints = [] 
    for i in range(exp.n()+1):
        if i == 0: continue
        pos_link = fk.fk(pos, i, positionOnly=True)
        constraints += [collision_constraint(pos_link, robot_size, obs[i:i+3], obs[i+3]) for i in range(0, n_obs*4, 4)]
    constraints = cd.vertcat(*constraints)

    # stage cost
    cost_stage = pr.w_input*cost_input + pr.w_pos*cost_goal + pr.w_coll*cost_coll + cost_joints + cost_vref * pr.w_vref

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

def panda_arm_params(exp: Experiment) -> np.ndarray:
    start = exp.initState()[0]
    goal = exp.goal().primeGoal().position()
    obs = np.array([[*o.position(), o.radius()] for o in exp.obstacles()]).flatten()
    r_body = [exp.rBody()]
    return np.concatenate((start, goal, obs, r_body))
