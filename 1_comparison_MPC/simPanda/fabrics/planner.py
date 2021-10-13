import casadi as ca
import numpy as np
import yaml

from optFabrics.planner.fabricPlanner import DefaultFabricPlanner
from optFabrics.planner.default_geometries import CollisionGeometry, GoalGeometry, LimitGeometry
from optFabrics.planner.default_energies import CollisionLagrangian, ExecutionLagrangian
from optFabrics.planner.default_maps import CollisionMap, LowerLimitMap, UpperLimitMap, SelfCollisionMap
from optFabrics.planner.default_leaves import defaultAttractor

from pandaFk import pandaFk


class FabricPlanner(object):
    def __init__(self, setupFile, n):
        self.parseSetup(setupFile)
        self._n = n
        self._planner = DefaultFabricPlanner(self._n, m_base=self._params['m_base'])
        self._q, self._qdot = self._planner.var()
        self._fks = []
        for i in range(0, self._n + 3):
            self._fks.append(pandaFk(self._q, i))

    def parseSetup(self, setupFile):
        with open(setupFile, "r") as stream:
            self._params = yaml.safe_load(stream)

    def addJointLimits(self, lower_limits, upper_limits):
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_col = CollisionLagrangian(x, xdot)
        geo_col = LimitGeometry(
            x, xdot,
            lam=self._params["limits"]["lam"],
            exp=self._params["limits"]["exp"]
        )
        for i in range(self._n):
            dm_col = UpperLimitMap(self._q, self._qdot, upper_limits[i], i)
            self._planner.addGeometry(dm_col, lag_col, geo_col)
            dm_col = LowerLimitMap(self._q, self._qdot, lower_limits[i], i)
            self._planner.addGeometry(dm_col, lag_col, geo_col)

    def addObstacles(self, obsts):
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_col = CollisionLagrangian(x, xdot)
        geo_col = CollisionGeometry(
            x, xdot,
            exp=self._params["obst"]["exp"],
            lam=self._params["obst"]["lam"]
        )
        for i, obst in enumerate(obsts):
            for fk in self._fks[3:]:
                dm_col = CollisionMap(self._q, self._qdot, fk, obst.x(), obst.r())
                self._planner.addGeometry(dm_col, lag_col, geo_col)

    def addSelfCollisionAvoidance(self):
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_selfCol = CollisionLagrangian(x, xdot)
        geo_selfCol = CollisionGeometry(
            x, xdot,
            exp=self._params["selfCol"]["exp"],
            lam=self._params["selfCol"]["lam"]
        )
        fks = self._fks
        for i in range(2, self._n+3):
            for j in range(i+2, self._n+3):
                dm_selfCol = SelfCollisionMap(self._q, self._qdot, fks[i], fks[j], self._params['selfCol']['r'])
                self._planner.addGeometry(dm_selfCol, lag_selfCol, geo_selfCol)

    def addGoal(self, goal):
        for subGoal in goal._goals:
            indices = subGoal.getIndices()
            fk = self._fks[subGoal._child_link][indices] - self._fks[subGoal._parent_link][indices]
            dm_psi, lag_psi, geo_psi, x_psi, xdot_psi = defaultAttractor(
                self._q, self._qdot, subGoal._desired_position, fk
            )
            geo_psi = GoalGeometry(
                x_psi, xdot_psi,
                k_psi=self._params['goal']['k_psi'] * subGoal._w
            )
            self._planner.addForcingGeometry(dm_psi, lag_psi, geo_psi)
            if subGoal.isPrimeGoal():
                self._x_psi = x_psi
                self._xdot_psi = xdot_psi
                self._dm_psi = dm_psi

    def concretize(self):
        # execution energy
        exLag = ExecutionLagrangian(self._q, self._qdot)
        self._planner.setExecutionEnergy(exLag)
        # Speed control
        ex_factor = self._params['speed']['ex_factor']
        self._planner.setDefaultSpeedControl(
            self._x_psi, self._dm_psi, exLag, ex_factor
        )
        self._planner.concretize()

    def computeAction(self, q, qdot):
        return self._planner.computeAction(q, qdot)
