import casadi as ca
import numpy as np

from optFabrics.planner.fabricPlanner import DefaultFabricPlanner
from optFabrics.planner.nonHolonomicPlanner import DefaultNonHolonomicPlanner
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
from optFabrics.diffGeometry.analyticSymbolicTrajectory import AnalyticSymbolicTrajectory


from optFabrics.diffGeometry.diffMap import DifferentialMap, RelativeDifferentialMap
from optFabrics.diffGeometry.energized_geometry import WeightedGeometry

from fabricsExperiments.generic.abstractPlanner import AbstractPlanner

from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle
from MotionPlanningEnv.sphereObstacle import SphereObstacle


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
        if self._exp.robotType() in ['groundRobot', 'boxer']:
            self._planner = DefaultNonHolonomicPlanner(self.n(), m_base=self.mBase(), debug=False)
        else:
            self._planner = DefaultFabricPlanner(self.n(), m_base=self.mBase(), debug=False)
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
            m_obst = obst.dim()
            if isinstance(obst, DynamicSphereObstacle):
                refTraj_i = AnalyticSymbolicTrajectory(ca.SX(np.identity(m_obst)), m_obst, traj=obst.traj())
                x_col = ca.SX.sym("x_col", m_obst)
                xdot_col = ca.SX.sym("xdot_col", m_obst)
                x_rel = ca.SX.sym("x_rel", m_obst)
                xdot_rel = ca.SX.sym("xdot_rel", m_obst)
            for j in range(1, self.n() + 1):
                if self._exp.robotType() == "pointMass" and j == 1:
                    continue
                if self._exp.robotType() == "groundRobot" and j == 1:
                    continue
                fk = self._exp.fk(self._q, j, positionOnly=True)
                if self._exp.robotType() == 'boxer':
                    fk = fk[0:2]
                if isinstance(obst, DynamicSphereObstacle):
                    phi_n = ca.norm_2(x_rel) / (obst.radius() + r_body) - 1
                    dm_n = DifferentialMap(phi_n, q=x_rel, qdot=xdot_rel)
                    dm_rel = RelativeDifferentialMap(q=x_col, qdot=xdot_col, refTraj =refTraj_i)
                    dm_col = DifferentialMap(fk, q=self._q, qdot=self._qdot)
                    eg = WeightedGeometry(g=geo_col, le=lag_col)
                    eg_n = eg.pull(dm_n)
                    eg_rel = eg_n.pull(dm_rel)
                    self._planner.addWeightedGeometry(dm_col, eg_rel)
                    #self._planner.addGeometry(dm_col, lag_col.pull(dm_n).pull(dm_rel), geo_col.pull(dm_n).pull(dm_rel))
                elif isinstance(obst, SphereObstacle):
                    """
                    dm_col = CollisionMap(
                        self._q, self._qdot, fk, obst.x(), obst.r(), r_body=r_body
                    )
                    """
                    dm_col = CollisionMap(
                        self._q, self._qdot, fk, obst.position(), obst.radius(), r_body=r_body
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
            if subGoal.type() == "staticJointSpaceSubGoal":
                fk = self._q[subGoal.indices()]
            else:
                fk_child = self._exp.fk(self._q, subGoal.childLink(), positionOnly=True)[subGoal.indices()]
                fk_parent = self._exp.fk(self._q, subGoal.parentLink(), positionOnly=True)[subGoal.indices()]
                fk = fk_child - fk_parent
            if subGoal.type() == "analyticSubGoal":
                goalSymbolicTraj = AnalyticSymbolicTrajectory(ca.SX(np.identity(subGoal.m())), subGoal.m(), traj=subGoal.traj())
                dm_psi, lag_psi, geo_psi, x_psi, xdot_psi = defaultDynamicAttractor(
                    self._q, self._qdot, fk, goalSymbolicTraj, k_psi=self.configAttractor()['k_psi'] * subGoal.weight()
                )
                self._planner.addForcingGeometry(dm_psi, lag_psi, geo_psi, goalVelocity=goalSymbolicTraj.xdot())
            elif subGoal.type() == "splineSubGoal": 
                goalSymbolicTraj = AnalyticSymbolicTrajectory(ca.SX(np.identity(subGoal.m())), subGoal.m(), traj=subGoal.traj())
                dm_psi, lag_psi, geo_psi, x_psi, xdot_psi = defaultDynamicAttractor(
                    self._q, self._qdot, fk, goalSymbolicTraj, k_psi=self.configAttractor()['k_psi'] * subGoal.weight()
                )
                self._planner.addForcingGeometry(dm_psi, lag_psi, geo_psi, goalVelocity=goalSymbolicTraj.xdot())
            else:
                dm_psi, lag_psi, geo_psi, x_psi, xdot_psi = defaultAttractor(
                    self._q, self._qdot, subGoal.position(), fk
                )
                geo_psi = GoalGeometry(
                    x_psi, xdot_psi, k_psi=self.configAttractor()["k_psi"] * subGoal.weight()
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
        """
        print("----")
        print(args[0])
        q = args[0]
        qdot = args[1]
        x_col, J_col, _ = self._dm_col.forward(q, qdot)
        xdot_col = np.dot(J_col, qdot)
        x_obst = args[2]
        xdot_obst = args[3]
        xddot_obst = args[4]
        x_rel, xdot_rel = self._dm_rel.forward(x_col, xdot_col, x_obst, xdot_obst, xddot_obst)
        x_n, J_n, Jdot_n = self._dm_n.forward(x_rel, xdot_rel)
        xdot_n = np.dot(J_n, xdot_rel)
        #print('fk : ', x_col)
        #print("x_rel : ", x_rel)
        #print("x_n : ", x_n)
        #geoEval = self._geo_col.evaluate(x_n, xdot_n)
        #print("h : ", geoEval[0])
        #print("xddot_n : ", geoEval[1])
        geoColEval = self._geo_col.evaluate(x_n, xdot_n)
        print("col")
        print(x_n)
        print(geoColEval)
        h_col = geoColEval[0]
        Jt_n = np.transpose(J_n)
        JtJ_inv = np.linalg.pinv(np.dot(Jt_n, J_n) + np.identity(3) * 1e-7)
        h_rel =np.dot(JtJ_inv, -np.dot(Jt_n, np.dot(Jdot_n, xdot_rel)) + np.dot(Jt_n, h_col))
        print("h_rel_man: ", h_rel)
        geoRelEval = self._geo_rel.evaluate(x_rel, xdot_rel)
        print("relative")
        print(x_rel)
        print(geoRelEval)
        print("differential map")
        #geoLeafEval = self._geo_leaf.evaluate(x_col, xdot_col, x_obst, xdot_obst, xddot_obst)
        #print(x_col)
        #print(geoLeafEval)
        #lagRelEval = self._lag_rel.evaluate(x_col, xdot_col, x_obst, xdot_obst, xddot_obst)
        #print(np.linalg.cond(lagRelEval[0]), lagRelEval[1], lagRelEval[2])
        debugEval = self._planner.debugEval(*args)
        #print(debugEval[:-1])
        #print(np.linalg.cond(debugEval[-1]))
        """
        action = self._planner.computeAction(*args)
        return action


if __name__ == "__main__":
    test_setup = "testSetupFiles/properFabric.yaml"
    myFabricPlanner = FabricPlanner(test_setup)
