import casadi as ca

from optFabrics.planner.fabricPlanner import DefaultFabricPlanner
from optFabrics.planner.default_geometries import CollisionGeometry, GoalGeometry
from optFabrics.planner.default_energies import CollisionLagrangian, ExecutionLagrangian
from optFabrics.planner.default_maps import CollisionMap
from optFabrics.planner.default_leaves import defaultAttractor

class FabricPlanner(object):
    def __init__(self):
        self._planner = DefaultFabricPlanner(2, m_base=1)
        self._q, self._qdot = self._planner.var()

    def addObstacles(self, obsts):
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_col = CollisionLagrangian(x, xdot)
        geo_col = CollisionGeometry(x, xdot, exp=1, lam=5)
        fks = [self._q]
        for i, obst in enumerate(obsts):
            for fk in fks:
                dm_col = CollisionMap(self._q, self._qdot, fk, obst.x(), obst.r())
                self._planner.addGeometry(dm_col, lag_col, geo_col)

    def addGoal(self, goal):
        fk = self._q
        self._dm_psi, lag_psi, _, self._x_psi, self._xdot_psi  = defaultAttractor(self._q, self._qdot, goal, fk)
        geo_psi = GoalGeometry(self._x_psi, self._xdot_psi, k_psi=5)
        self._planner.addForcingGeometry(self._dm_psi, lag_psi, geo_psi)

    def concretize(self):
        # execution energy
        exLag = ExecutionLagrangian(self._q, self._qdot)
        self._planner.setExecutionEnergy(exLag)
        # Speed control
        ex_factor = 1.0
        self._planner.setDefaultSpeedControl(self._x_psi, self._dm_psi, exLag, ex_factor)
        self._planner.concretize()

    def computeAction(self, q, qdot):
        return self._planner.computeAction(q, qdot)

