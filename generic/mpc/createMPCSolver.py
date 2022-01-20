import numpy as np
import forcespro
import forcespro.nlp
from forcespro import MultistageProblem
import casadi as ca
from casadi import sin, cos, dot
from urdfpy import URDF

import os

from forwardkinematics.fksCommon.fk_creator import FkCreator
from fabricsExperiments.generic.mpc.parameterMap import getParameterMap

slack = True
n = 7
m = 3
dt = 0.1
nbObst = 5
m_obst = 3
robotType = "panda"
pairs = []
if robotType == 'panda':
    pairs = [
        #[0, 3],
        #[0, 4],
        #[0, 5],
        [0, 6],
        [0, 7],
        #[1, 3],
        #[1, 4],
        #[1, 5],
        [1, 6],
        [1, 7],
        [2, 6],
        [2, 7],
    ]
elif robotType == 'planarArm':
    for i in range(n+1):
        for j in range(i+2, n+1):
            pairs.append([i, j])
generic_fk = FkCreator(robotType, n).fk()
paramMap, npar, nx, nu, ns = getParameterMap(n, m, nbObst, m_obst, slack)
N = 30
dt_str = str(dt).replace(".", "")

# file names
dt_str = str(dt).replace(".", "")
# TODO: How to define the path to the generated solver? <26-10-21, mspahn> #
if slack:
    solverName = "solver_n" + str(n) + "_" + dt_str + "_H" + str(N)
else:
    solverName = "solver_n" + str(n) + "_" + dt_str + "_H" + str(N) + "_noSlack"


# MPC parameter
upper_lim = np.ones(n) * np.inf
lower_lim = np.ones(n) * -np.inf
limitVel = np.ones(n) * 400
limitAcc = np.ones(n) * 100
if slack:
    slack_upper = np.array([np.inf])
    slack_lower = np.array([0])
    xu = np.concatenate((upper_lim, limitVel, slack_upper))
    xl = np.concatenate((lower_lim, -limitVel, slack_lower))
else:
    xu = np.concatenate((upper_lim, limitVel))
    xl = np.concatenate((lower_lim, -limitVel))
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
    qddot = z[nx + ns : ns + nx + nu]
    w = p[paramMap["w"]]
    wvel = p[paramMap["wvel"]]
    wu = p[paramMap["wu"]]
    g = p[paramMap["g"]]
    W = diagSX(w, m)
    Wvel = diagSX(wvel, n)
    Wu = diagSX(wu, n)
    fk = generic_fk.fk(q, n, positionOnly=True)
    err = fk - g
    Jx = ca.dot(err, ca.mtimes(W, err)) # only penalize in objN -> does not affect the result
    Jvel = ca.dot(qdot, ca.mtimes(Wvel, qdot))
    Ju = ca.dot(qddot, ca.mtimes(Wu, qddot))
    if slack:
        s = z[nx]
        ws = p[paramMap["ws"]]
        J = Jx + Jvel + Ju + ws * s**2
    else:
        J = Jx + Jvel + Ju
    return J


def eval_objN(z, p):
    q = z[0:n]
    qdot = z[n:nx]
    w = p[paramMap["w"]]
    wvel = p[paramMap["wvel"]]
    g = p[paramMap["g"]]
    W = diagSX(w, m)
    Wvel = diagSX(wvel, n)
    fk = generic_fk.fk(q, n, positionOnly=True)
    err = fk - g
    Jx = ca.dot(err, ca.mtimes(W, err)) # only have that term here
    Jvel = ca.dot(qdot, ca.mtimes(Wvel, qdot))
    if slack:
        s = z[nx]
        ws = p[paramMap["ws"]]
        J = Jx + Jvel + ws * s**2
    else:
        J = Jx  + Jvel
    return J


def eval_ineq(z, p):
    q = z[0:n]
    qdot = z[n:nx]
    if slack:
        s = z[nx]
    else:
        s = 0.0
    qddot = z[nx + ns : ns + nx + nu]
    obsts = p[paramMap["obst"]]
    r_body = p[paramMap["r_body"]]
    ineqs = []
    for j in range(n):
        fk = generic_fk.fk(q, j + 1, positionOnly=True)
        for i in range(nbObst):
            obst = obsts[i * (m_obst + 1) : (i + 1) * (m_obst + 1)]
            x = obst[0:m_obst]
            r = obst[m_obst]
            dist = ca.norm_2(fk - x)
            ineqs.append(dist - r - r_body + s)
    all_ineqs = ineqs + eval_jointLimits(z, p) + eval_selfCollision(z, p)
    return all_ineqs


def eval_selfCollision(z, p):
    if slack:
        s = z[nx]
    else:
        s = 0.0
    q = z[0:n]
    r_body = p[paramMap["r_body"]]
    ineqs = []
    for pair in pairs:
        fk1 = generic_fk.fk(q, pair[0], positionOnly=True)
        fk2 = generic_fk.fk(q, pair[1], positionOnly=True)
        dist = ca.norm_2(fk1 - fk2)
        ineqs.append(dist - (2 * r_body) + s)
    return ineqs


def eval_jointLimits(z, p):
    # Parameters in state boundaries?
    q = z[0:n]
    if slack:
        s = z[nx]
    else:
        s = 0.0
    lower_limits = p[paramMap["lower_limits"]]
    upper_limits = p[paramMap["upper_limits"]]
    ineqs = []
    for j in range(n):
        dist_lower = q[j] - lower_limits[j]
        dist_upper = upper_limits[j] - q[j]
        ineqs.append(dist_lower + s)
        ineqs.append(dist_upper + s)
    return ineqs


def continuous_dynamics(x, u):
    q = x[0:n]
    qdot = x[n:nx]
    qddot = u[ns : n + ns]
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
    codeoptions.nlp.integrator.type = "ERK2" # Consider Bruno's implementation on dynamics 
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
