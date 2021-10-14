import casadi as ca
import yaml

from optFabrics.planner.fabricPlanner import DefaultFabricPlanner
from optFabrics.planner.default_geometries import CollisionGeometry, GoalGeometry
from optFabrics.planner.default_energies import CollisionLagrangian, ExecutionLagrangian
from optFabrics.planner.default_maps import CollisionMap, LowerLimitMap, UpperLimitMap, SelfCollisionMap
from optFabrics.planner.default_leaves import defaultAttractor
from optFabrics.planner.default_geometries import CollisionGeometry, GoalGeometry, LimitGeometry


from fabricsExperiments.infrastructure.abstractPlanner import AbstractPlanner


class FabricPlanner(AbstractPlanner):
    def __init__(self, exp, setupFile):
        required_keys = ['type', 'n', 'obst', 'attractor', 'speed', 'damper', 'limits', 'selfCol']
        super().__init__(exp, setupFile, required_keys)
        self.reset()

    def reset(self):
        print("RESETTING PLANNER")
        self._planner = DefaultFabricPlanner(self.n(), m_base=self.mBase())
        self._q, self._qdot = self._planner.var()

    def n(self):
        return self._setup['n']

    def mBase(self):
        return self._setup['m_base']

    def configObst(self):
        return self._setup['obst']

    def configSelfColAvoidance(self):
        return self._setup['selfCol']

    def configAttractor(self):
        return self._setup['attractor']

    def configSpeed(self):
        return self._setup['speed']

    def configLimits(self):
        return self._setup['limits']

    def configDamper(self):
        return self._setup['damper']

    def setObstacles(self, obsts, r_body):
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_col = CollisionLagrangian(x, xdot)
        geo_col = CollisionGeometry(
            x, xdot,
            exp=self.configObst()["exp"],
            lam=self.configObst()["lam"]
        )
        for i, obst in enumerate(obsts):
            for i in range(1, self.n()):
                fk = self._exp.fk(self._q, i, positionOnly=True)
                dm_col = CollisionMap(self._q, self._qdot, fk, obst.x(), obst.r(), r_body=r_body)
                self._planner.addGeometry(dm_col, lag_col, geo_col)

    def setSelfCollisionAvoidance(self, r_body):
        if self._exp.robotType() == 'pointMass':
            return
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_selfCol = CollisionLagrangian(x, xdot)
        geo_selfCol = CollisionGeometry(
            x, xdot,
            exp=self.configSelfColAvoidance()['exp'],
            lam=self.configSelfColAvoidance()['lam'],
        )
        for i in range(self.n()+1):
            for j in range(i+2, self.n()+1):
                fk_i = self._exp.fk(self._q, i, positionOnly=True)
                fk_j = self._exp.fk(self._q, j, positionOnly=True)
                dm_selfCol = SelfCollisionMap(self._q, self._qdot, fk_i, fk_j, r_body)
                self._planner.addGeometry(dm_selfCol, lag_selfCol, geo_selfCol)

    def setJointLimits(self, limits):
        x = ca.SX.sym("x", 1)
        xdot = ca.SX.sym("xdot", 1)
        lag_col = CollisionLagrangian(x, xdot)
        geo_col = LimitGeometry(
            x, xdot,
            lam=self.configLimits()["lam"],
            exp=self.configLimits()["exp"],
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
            fk =  fk_child - fk_parent
            dm_psi, lag_psi, geo_psi, x_psi, xdot_psi = defaultAttractor(
                self._q, self._qdot, subGoal.desiredPosition(), fk
            )
            geo_psi = GoalGeometry(
                x_psi, xdot_psi,
                k_psi=self.configAttractor()['k_psi'] * subGoal.w()
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
        ex_factor = self.configSpeed()['ex_factor']
        self._planner.setDefaultSpeedControl(
            self._x_psi, self._dm_psi, exLag, ex_factor
        )
        self._planner.concretize()

    def computeAction(self, q, qdot):
        return self._planner.computeAction(q, qdot)

if __name__ == "__main__":
    test_setup = "testSetupFiles/properFabric.yaml"
    myFabricPlanner = FabricPlanner(test_setup)
