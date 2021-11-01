import numpy as np
import forcespro
import forcespro.nlp
from forcespro import MultistageProblem
import casadi as ca
from casadi import sin, cos, dot
from urdfpy import URDF

import os
from scipy.integrate import odeint

from casadiFk import ForwardKinematics
from fabricsExperiments.generic.mpc.parameterMap import getParameterMap

n = 3
m = 2
dt = 0.5
nbObst = 5
m_obst = 2
robotType = "planarArm"
pairs = []
if robotType == 'panda':
    pairs = [
        [0, 3],
        [0, 4],
        [0, 5],
        [0, 6],
        [0, 7],
        [1, 3],
        [1, 4],
        [1, 5],
        [1, 6],
        [1, 7],
        [2, 6],
        [2, 7],
    ]
elif robotType == 'planarArm':
    for i in range(n+1):
        for j in range(i+2, n+1):
            pairs.append([i, j])
generic_fk = ForwardKinematics(robot_type=robotType)
paramMap, npar, nx, nu, ns = getParameterMap(n, m, nbObst, m_obst)
N = 20
dt_str = str(dt).replace(".", "")

# file names
dt_str = str(dt).replace(".", "")
# TODO: How to define the path to the generated solver? <26-10-21, mspahn> #
solverName = "solver_n" + str(n) + "_" + dt_str + "_H" + str(N)


# MPC parameter
upper_lim = np.ones(n) * np.inf
lower_lim = np.ones(n) * -np.inf
limitVel = np.ones(n) * 4
limitAcc = np.ones(n) * 1
slack_upper = np.array([np.inf])
slack_lower = np.array([0])
xu = np.concatenate((upper_lim, limitVel, slack_upper))
xl = np.concatenate((lower_lim, -limitVel, slack_lower))
uu = limitAcc
ul = -uu


def diagSX(val, size):
    a = ca.SX(size, size)
    for i in range(size):
        a[i, i] = val[i]
    return a


def eval_obj(z, p):
    q = z[0:n]
    qdot = z[n:nx]
    slack = z[nx]
    qddot = z[nx + ns : ns + nx + nu]
    w = p[paramMap["w"]]
    wvel = p[paramMap["wvel"]]
    wu = p[paramMap["wu"]]
    ws = p[paramMap["ws"]]
    g = p[paramMap["g"]]
    W = diagSX(w, m)
    Wvel = diagSX(wvel, n)
    Wu = diagSX(wu, n)
    fk = generic_fk.getFk(q, n, positionOnly=True)
    err = fk - g
    Jx = ca.dot(err, ca.mtimes(W, err))
    Jvel = ca.dot(qdot, ca.mtimes(Wvel, qdot))
    Ju = ca.dot(qddot, ca.mtimes(Wu, qddot))
    J = Jx + Jvel + ws * slack
    return J


def eval_objN(z, p):
    q = z[0:n]
    qdot = z[n:nx]
    slack = z[nx]
    qddot = z[nx + ns : ns + nx + nu]
    w = p[paramMap["w"]]
    wvel = p[paramMap["wvel"]]
    ws = p[paramMap["ws"]]
    g = p[paramMap["g"]]
    W = diagSX(w, m)
    Wvel = diagSX(wvel, n)
    fk = generic_fk.getFk(q, n, positionOnly=True)
    err = fk - g
    Jx = ca.dot(err, ca.mtimes(W, err))
    Jvel = ca.dot(qdot, ca.mtimes(Wvel, qdot))
    J = 1 * Jx + ws * slack
    return J


def eval_ineq(z, p):
    q = z[0:n]
    qdot = z[n:nx]
    slack = z[nx]
    qddot = z[nx + ns : ns + nx + nu]
    obsts = p[paramMap["obst"]]
    r_body = p[paramMap["r_body"]]
    ineqs = []
    for j in range(n):
        fk = generic_fk.getFk(q, j + 1, positionOnly=True)
        for i in range(nbObst):
            obst = obsts[i * (m_obst + 1) : (i + 1) * (m_obst + 1)]
            x = obst[0:m_obst]
            r = obst[m_obst]
            dist = ca.norm_2(fk - x)
            ineqs.append(dist - r - r_body + slack)
    all_ineqs = ineqs + eval_jointLimits(z, p) + eval_selfCollision(z, p)
    return all_ineqs


def eval_selfCollision(z, p):
    slack = z[nx]
    q = z[0:n]
    r_body = p[paramMap["r_body"]]
    ineqs = []
    for pair in pairs:
        fk1 = generic_fk.getFk(q, pair[0], positionOnly=True)
        fk2 = generic_fk.getFk(q, pair[1], positionOnly=True)
        dist = ca.norm_2(fk1 - fk2)
        ineqs.append(dist - (2 * r_body) + slack)
    return ineqs


def eval_jointLimits(z, p):
    q = z[0:n]
    slack = z[nx]
    lower_limits = p[paramMap["lower_limits"]]
    upper_limits = p[paramMap["upper_limits"]]
    ineqs = []
    for j in range(n):
        dist_lower = q[j] - lower_limits[j]
        dist_upper = upper_limits[j] - q[j]
        ineqs.append(dist_lower + slack)
        ineqs.append(dist_upper + slack)
    return ineqs


def continuous_dynamics(x, u):
    q = x[0:n]
    qdot = x[n:nx]
    qddot = u[1 : n + 1]
    acc = ca.vertcat(qdot, qddot)
    return acc


def main():
    model = forcespro.nlp.SymbolicModel(N)
    model.continuous_dynamics = continuous_dynamics
    model.objective = eval_obj
    E1 = np.concatenate([np.eye(nx), np.zeros((nx, nu + ns))], axis=1)
    # E2 = np.concatenate((E1, np.zeros((1, nx + nu))))
    model.E = E1
    model.lb = np.concatenate((xl, ul))
    model.ub = np.concatenate((xu, uu))
    model.npar = npar
    model.nvar = nx + nu + ns
    model.neq = nx
    nself = len(pairs)
    model.nh = nbObst * n + 2 * n + nself
    model.hu = np.ones(nbObst * n + 2 * n + nself) * np.inf
    model.hl = np.zeros(nbObst * n + 2 * n + nself)
    model.ineq = eval_ineq
    model.objectiveN = eval_objN
    model.xinitidx = range(0, nx)

    # Get the default solver options
    codeoptions = forcespro.CodeOptions(solverName)
    codeoptions.printlevel = 0
    codeoptions.optlevel = 3
    codeoptions.nlp.integrator.type = "ERK2"
    codeoptions.nlp.integrator.Ts = dt
    codeoptions.nlp.integrator.nodes = 5
    # codeoptions.maxit = 300
    # codeoptions.solver_timeout = -1
    # codeoptions.nlp.TolStat = -1 # default 1e-5
    # codeoptions.nlp.TolEq = -1 # default 1e-6
    # codeoptions.nlp.TolIneq = -1 # default 1e-6
    # codeoptions.nlp.integrator.type = "ERK2"
    # codeoptions.nlp.integrator.Ts = 0.1
    # codeoptions.nlp.integrator.nodes = 5
    # Generate solver for previously initialized model
    _ = model.generate_solver(codeoptions)


if __name__ == "__main__":
    main()
