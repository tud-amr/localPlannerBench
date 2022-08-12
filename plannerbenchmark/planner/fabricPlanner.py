# General dependencies
from dataclasses import dataclass, field
from typing import Dict
import numpy as np

# Dependencies for this specific planner
from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner

# Motion planning scenes
from MotionPlanningEnv.dynamicSphereObstacle import DynamicSphereObstacle
from MotionPlanningEnv.sphereObstacle import SphereObstacle
from MotionPlanningGoal.dynamicSubGoal import DynamicSubGoal

# Dependencies on plannerbenchmark
from plannerbenchmark.generic.planner import Planner, PlannerConfig

@dataclass
class FabricConfig(PlannerConfig):
    m_base: float = 0.2
    m_ratio: float = 1.0
    obst: Dict[str, float] = field(default_factory = lambda: ({'exp': 1.0, 'lam': 1.0}))
    selfCol: Dict[str, float] = field(default_factory = lambda: ({'exp': 1.0, 'lam': 1.0}))
    attractor: Dict[str, float] = field(default_factory = lambda: ({'k_psi': 3.0}))
    speed: Dict[str, float] = field(default_factory = lambda : ({'ex_factor': 1.0}))
    dynamic: bool = False
    limits: Dict[str, float] = field(default_factory = lambda: ({'exp': 1.0, 'lam': 1.0}))
    damper: Dict[str, object] = field(default_factory = lambda :({'r_d': 0.8, 'b': [0.1, 6.5]}))
    l_offset: float = 0.2
    m_arm: float = 1.0
    m_rot: float = 1.0


class FabricPlanner(Planner):

    def __init__(self, exp, **kwargs):
        super().__init__(exp, **kwargs)
        self._config = FabricConfig(**kwargs)
        self.reset()
        base_energy: str = (
            "0.5 * sym('base_inertia') * ca.dot(xdot, xdot)"
        )
        collision_geometry: str = (
            "-sym('obst_geo_lam') / (x ** sym('obst_geo_exp')) * xdot ** 2"
        )
        collision_finsler: str = (
            "1.0/(x**1) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
        )
        limit_geometry: str = (
            "-0.1 / (x ** 1) * xdot ** 2"
        )
        limit_finsler: str = (
            "1.0/(x**1) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
        )
        self_collision_geometry: str = (
            "-sym('self_geo_lam') / (x ** sym('self_geo_exp')) * xdot ** 2"
        )
        self_collision_finsler: str = (
            "1.0/(x**1) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
        )
        attractor_potential: str = (
            "5.0 * (ca.norm_2(x) + 1 / 10 * ca.log(1 + ca.exp(-2 * 10 * ca.norm_2(x))))"
        )
        attractor_metric: str = (
            "((2.0 - 0.3) * ca.exp(-1 * (0.75 * ca.norm_2(x))**2) + 0.3) * ca.SX(np.identity(x.size()[0]))"
        )
        self._planner = ParameterizedFabricPlanner(
            self.config.n,
            self._exp.robotType(),
            base_energy=base_energy,
            collision_geometry=collision_geometry,
            collision_finsler=collision_finsler,
            self_collision_geometry=self_collision_geometry,
            self_collision_finsler=self_collision_finsler,
        )
        self._collision_links = [i for i in range(1, self.config.n+1)]
        self._number_obstacles = 0
        self._dynamic_goal = False

    def initialize_runtime_arguments(self):
        self._runtime_arguments = {}
        self._runtime_arguments['base_inertia'] = np.array([self.config.m_base])
        for j in range(self._number_static_obstacles):
            self._runtime_arguments[f'radius_obst_{j}'] = np.array([self._static_obsts[j].radius()])
            self._runtime_arguments[f'x_obst_{j}'] = np.array(self._static_obsts[j].position())
            for i in self._collision_links:
                self._runtime_arguments[f'obst_geo_exp_obst_{j}_{i}_leaf'] = np.array([self.config.obst['exp']])
                self._runtime_arguments[f'obst_geo_lam_obst_{j}_{i}_leaf'] = np.array([self.config.obst['lam']])
                self._runtime_arguments[f'radius_body_{i}'] = np.array([self._r_body])
        for j in range(self._number_dynamic_obstacles):
            self._runtime_arguments[f'radius_dynamic_obst_{j}'] = np.array([self._dynamic_obsts[j].radius()])
            for i in self._collision_links:
                self._runtime_arguments[f'obst_geo_exp_dynamic_obst_{j}_{i}_leaf'] = np.array([self.config.obst['exp']])
                self._runtime_arguments[f'obst_geo_lam_dynamic_obst_{j}_{i}_leaf'] = np.array([self.config.obst['lam']])
                self._runtime_arguments[f'radius_body_{i}'] = np.array([self._r_body])
        for link, paired_links in self._self_collision_dict.items():
            for paired_link in paired_links:
                self._runtime_arguments[f'self_geo_lam_self_collision_{link}_{paired_link}'] = np.array([self.config.selfCol['lam']])
                self._runtime_arguments[f'self_geo_exp_self_collision_{link}_{paired_link}'] = np.array([self.config.selfCol['exp']])

    def setObstacles(self, obsts, r_body):
        self._dynamic_obsts = []
        self._static_obsts = []
        for obst in obsts:
            if isinstance(obst, DynamicSphereObstacle):
                self._dynamic_obsts.append(obst)
            else:
                self._static_obsts.append(obst)
        self._number_static_obstacles = len(self._static_obsts)
        self._number_dynamic_obstacles = len(self._dynamic_obsts)

    def setSelfCollisionAvoidance(self, r_body):
        self_collision_pairs = self._exp.selfCollisionPairs()
        self._self_collision_dict = {}
        for pair in self_collision_pairs:
            if pair[0] in self._self_collision_dict.keys():
                self._self_collision_dict[pair[0]].append(pair[1])
            else:
                self._self_collision_dict[pair[0]] = [pair[1]]
        self._r_body = r_body

    def setJointLimits(self, limits):
        self._limits = limits

    def setGoal(self, goal):
        self._dynamic_goal = isinstance(goal.primeGoal(), DynamicSubGoal) 
        self._goal = goal

    def concretize(self):
        self._planner.set_components(
            self._collision_links,
            self._self_collision_dict,
            self._goal,
            limits=self._limits,
            number_obstacles=self._number_static_obstacles,
            number_dynamic_obstacles=self._number_dynamic_obstacles,
        )
        self._planner.concretize()
        self.initialize_runtime_arguments()


    def adapt_runtime_arguments(self, args):
        self._runtime_arguments['q'] = args[0]
        self._runtime_arguments['qdot'] = args[1]
        if self._dynamic_goal:
            self._runtime_arguments['x_ref_goal_0_leaf'] = args[2]
            self._runtime_arguments['xdot_ref_goal_0_leaf'] = args[3]
            self._runtime_arguments['xddot_ref_goal_0_leaf'] = args[4]
        else:
            for i, sub_goal in enumerate(self._goal.subGoals()):
                self._runtime_arguments[f'x_goal_{i}'] = np.array(sub_goal.position())
                self._runtime_arguments[f'weight_goal_{i}'] = np.array(sub_goal.weight() * self.config.attractor['k_psi'])
        for i, obst in enumerate(self._dynamic_obsts):
            for j in self._collision_links:
                self._runtime_arguments[f'x_ref_dynamic_obst_{i}_{j}_leaf'] = args[1 + 3*i+1]
                self._runtime_arguments[f'xdot_ref_dynamic_obst_{i}_{j}_leaf'] = args[1 + 3*i + 2]
                self._runtime_arguments[f'xddot_ref_dynamic_obst_{i}_{j}_leaf'] = args[1 + 3*i + 3]


        pass

    def computeAction(self, *args):
        self.adapt_runtime_arguments(args)
        action = self._planner.compute_action(**self._runtime_arguments)
        return action
