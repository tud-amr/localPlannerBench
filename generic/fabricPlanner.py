import casadi as ca
import yaml
import numpy as np

from optFabrics.planner.fabricPlanner import DefaultFabricPlanner
from optFabrics.planner.default_geometries import CollisionGeometry, GoalGeometry
from optFabrics.planner.default_energies import CollisionLagrangian, ExecutionLagrangian
from optFabrics.planner.default_maps import (
    CollisionMap,
    LowerLimitMap,
    UpperLimitMap,
    SelfCollisionMap,
)
from optFabrics.planner.default_leaves import defaultAttractor, defaultDynamicAttractor
from optFabrics.planner.default_geometries import (
    CollisionGeometry,
    GoalGeometry,
    LimitGeometry,
)

from optFabrics.diffGeometry.referenceTrajectory import AnalyticTrajectory

from optFabrics.diffGeometry.diffMap import DifferentialMap, RelativeDifferentialMap

from fabricsExperiments.generic.abstractPlanner import AbstractPlanner
from fabricsExperiments.infrastructure.variables import t

from obstacle import RefDynamicObstacle


class FabricPlanner(AbstractPlanner):
    def __init__(self, exp, setupFile):
        required_keys = [
            "type",
            "n",
            "obst",
            "attractor",
            "speed",
            "damper",
            "limits",
            "selfCol",
            "interval",
            "dynamic",
        ]
        super().__init__(exp, setupFile, required_keys)
        self.reset()

    def reset(self):
        self._planner = DefaultFabricPlanner(self.n(), m_base=self.mBase())
        self._q, self._qdot = self._planner.var()

    def interval(self):
        return self._setup["interval"]

    def n(self):
        return self._setup["n"]

    def mBase(self):
        return self._setup["m_base"]

    def dynamic(self):
        return self._setup["dynamic"]

    def configObst(self):
        return self._setup["obst"]

    def configSelfColAvoidance(self):
        return self._setup["selfCol"]

    def configAttractor(self):
        return self._setup["attractor"]

    def configSpeed(self):
        return self._setup["speed"]

    def configLimits(self):
        return self._setup["limits"]

    def configDamper(self):
        return self._setup["damper"]

    def setObstacles(self, obsts, r_body):
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_col = CollisionLagrangian(x, xdot)
        geo_col = CollisionGeometry(
            x, xdot, exp=self.configObst()["exp"], lam=self.configObst()["lam"]
        )
        for i, obst in enumerate(obsts):
            x_col = ca.SX.sym("x_col", 2)
            xdot_col = ca.SX.sym("xdot_col", 2)
            x_rel = ca.SX.sym("x_rel", 2)
            xdot_rel = ca.SX.sym("xdot_rel", 2)
            for j in range(1, self.n() + 1):
                if self._exp.robotType() == "pointMass" and j == 1:
                    continue
                fk = self._exp.fk(self._q, j, positionOnly=True)
                if isinstance(obst, RefDynamicObstacle):
                    phi_n = ca.norm_2(x_rel) / obst.r() - 1
                    dm_n = DifferentialMap(phi_n, q=x_rel, qdot=xdot_rel)
                    dm_rel = RelativeDifferentialMap(q=x_col, qdot=xdot_col, refTraj = obst.refTraj())
                    dm_col = DifferentialMap(fk, q=self._q, qdot=self._qdot)
                    self._planner.addGeometry(dm_col, lag_col.pull(dm_n).pull(dm_rel), geo_col.pull(dm_n).pull(dm_rel))
                else:
                    dm_col = CollisionMap(
                        self._q, self._qdot, fk, obst.x(), obst.r(), r_body=r_body
                    )
                    self._planner.addGeometry(dm_col, lag_col, geo_col)

    def setSelfCollisionAvoidance(self, r_body):
        if self._exp.robotType() == "pointMass":
            return
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_selfCol = CollisionLagrangian(x, xdot)
        geo_selfCol = CollisionGeometry(
            x,
            xdot,
            exp=self.configSelfColAvoidance()["exp"],
            lam=self.configSelfColAvoidance()["lam"],
        )
        for pair in self._exp.selfCollisionPairs():
            fk_1 = self._exp.fk(self._q, pair[0], positionOnly=True)
            fk_2 = self._exp.fk(self._q, pair[1], positionOnly=True)
            dm_selfCol = SelfCollisionMap(self._q, self._qdot, fk_1, fk_2, r_body)
            self._planner.addGeometry(dm_selfCol, lag_selfCol, geo_selfCol)

    def setJointLimits(self, limits):
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_col = CollisionLagrangian(x, xdot)
        geo_col = LimitGeometry(
            x, xdot, lam=self.configLimits()["lam"], exp=self.configLimits()["exp"],
        )
        for i in range(self.n()):
            dm_col = UpperLimitMap(self._q, self._qdot, limits[1][i], i)
            self._planner.addGeometry(dm_col, lag_col, geo_col)
            dm_col = LowerLimitMap(self._q, self._qdot, limits[0][i], i)
            self._planner.addGeometry(dm_col, lag_col, geo_col)

    def setGoal(self, goal):
        for subGoal in goal.subGoals():
            fk_child = self._exp.fk(self._q, subGoal.childLink())[subGoal.indices()]
            fk_parent = self._exp.fk(self._q, subGoal.parentLink())[subGoal.indices()]
            fk = fk_child - fk_parent
            if subGoal.dynamic():
                refTraj = subGoal.trajectory()
                dm_psi, lag_psi, geo_psi, x_psi, xdot_psi = defaultDynamicAttractor(
                    self._q, self._qdot, fk, refTraj,
                    k_psi=self.configAttractor()['k_psi'] * subGoal.w()
                )
                self._planner.addForcingGeometry(
                    dm_psi, lag_psi, geo_psi, goalVelocity=refTraj.xdot()
                )
            else:
                dm_psi, lag_psi, geo_psi, x_psi, xdot_psi = defaultAttractor(
                    self._q, self._qdot, subGoal.desiredPosition(), fk
                )
                geo_psi = GoalGeometry(
                    x_psi, xdot_psi, k_psi=self.configAttractor()["k_psi"] * subGoal.w()
                )
                self._planner.addForcingGeometry(dm_psi, lag_psi, geo_psi)
            if subGoal.isPrimeGoal():
                self._dm_psi = dm_psi
                self._x_psi = x_psi
                self._xdot_psi = xdot_psi

    def concretize(self):
        # execution energy
        exLag = ExecutionLagrangian(self._q, self._qdot)
        self._planner.setExecutionEnergy(exLag)
        # Speed control
        ex_factor = self.configSpeed()["ex_factor"]
        self._planner.setDefaultSpeedControl(
            self._x_psi,
            self._dm_psi,
            exLag,
            ex_factor,
            r_b=self.configDamper()["r_d"],
            b=self.configDamper()["b"],
        )
        self._planner.concretize()

    def computeAction(self, *args):
        return self._planner.computeAction(*args)


if __name__ == "__main__":
    test_setup = "testSetupFiles/properFabric.yaml"
    myFabricPlanner = FabricPlanner(test_setup)
