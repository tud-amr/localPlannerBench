import numpy
import ctypes

name = "solver_n2_05_H2_noSlack"
requires_callback = True
lib = "lib/libsolver_n2_05_H2_noSlack.so"
lib_static = "lib/libsolver_n2_05_H2_noSlack.a"
c_header = "include/solver_n2_05_H2_noSlack.h"
nstages = 2

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, ( 12,   1),   12),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  4,   1),    4),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, ( 58,   1),   58)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x1"                  , ""      , ""               , ctypes.c_double, numpy.float64,     (  6,),    6),
 ("x2"                  , ""      , ""               , ctypes.c_double, numpy.float64,     (  6,),    6)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
('it2opt', ctypes.c_int),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('rsnorm', ctypes.c_double),
('rcompnorm', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int),
('lsit_cc', ctypes.c_int),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double),
('fevalstime', ctypes.c_double)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(6, 4, 14, 29, 2, 2, 14, 0), 
	(6, 0, 14, 29, 6, 6, 14, 0)
]