import casadi as cd
import numpy as np

# Cost terms 
def l2_normalized(pos: cd.SX, start: np.ndarray, goal:np.ndarray) -> cd.SX:
    start_to_goal = (goal - start).T @ (goal - start)
    start_to_goal = cd.fmax(start_to_goal, 1) # clip normalization
    pos_to_goal = (goal - pos).T @ (goal - pos)
    return pos_to_goal / start_to_goal 

def custom_collision_potential(pos: cd.SX, robot_size: int, obs_pos: np.ndarray, obs_size: int) -> cd.SX:
    ell_axis = robot_size + obs_size
    d_vec = obs_pos - pos 
    d_vec_ell = d_vec / ell_axis
    d = d_vec_ell.T @ d_vec_ell

    # NOTE: solver crashes more often with this exponent
    #return 1.0 / (1.0 + cd.exp(d - 1.6))

    return 1 / (1 + 2*d)

def input_normalized(u: cd.SX, limit: np.ndarray) -> cd.SX:
    u_normalized = u/limit
    cost = u_normalized.T @ u_normalized
    cost_normalized = cost / u.shape[0]
    return cost_normalized

# Constraints
def collision_constraint(pos: cd.SX, robot_size: int, obs_pos: np.ndarray, obs_size: int) -> cd.SX:
    safety_factor = 1.10
    pos_to_obs = ((obs_pos- pos).T @ (obs_pos- pos))**0.5
    constraint_coll = pos_to_obs - (robot_size + obs_size)*safety_factor

    return constraint_coll
