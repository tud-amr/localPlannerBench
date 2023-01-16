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
class SensorFabricConfig(PlannerConfig):
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
    number_lidar_rays: int = 24
    radius_ray_obstacles: float = 0.1


class SensorFabricPlanner(Planner):

    def __init__(self, exp, **kwargs):
        super().__init__(exp, **kwargs)
        self._config = SensorFabricConfig(**kwargs)
        self.reset()
        base_energy: str = (
            "0.5 * sym('base_inertia') * ca.dot(xdot, xdot)"
        )
        collision_geometry: str = (
            "-2*sym('obst_geo_lam') / (x**sym('obst_geo_exp')) * (1 - ca.heaviside(xdot)) * xdot**2"
        )
        collision_finsler: str = (
            f"(20.0/{self._config.number_lidar_rays}) / (x**2) * (1 - ca.heaviside(xdot)) * xdot**2"
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
            attractor_potential=attractor_potential,
            attractor_metric=attractor_metric,
        )
        self._collision_links = [i for i in range(1, self.config.n+1)]
        self._collision_links = [1]
        self._number_static_obstacles = self._config.number_lidar_rays
        self._dynamic_goal = False

    def initialize_runtime_arguments(self):
        self._runtime_arguments = {}
        self._runtime_arguments['weight_goal_0'] = np.array(self.config.attractor['k_psi'])
        self._runtime_arguments['base_inertia'] = np.array([self.config.m_base])
        for j in range(self._number_static_obstacles):
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
        self._number_dynamic_obstacles = 0

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
        self._dynamic_goal = isinstance(goal.primary_goal(), DynamicSubGoal)
        self._goal = goal

    def concretize(self):
        self._planner.set_components(
            self._collision_links,
            self._self_collision_dict,
            self._goal,
            limits=self._limits,
            number_obstacles=self._config.number_lidar_rays,
            number_dynamic_obstacles=0,
        )
        self._planner.concretize()
        self.initialize_runtime_arguments()

    def adapt_runtime_arguments(self, args):
        time = args[3]
        self._runtime_arguments['q'] = args[0]
        self._runtime_arguments['qdot'] = args[1]
        if self._dynamic_goal:
            self._runtime_arguments['x_ref_goal_0_leaf'] = np.array(self._goal.primary_goal().position(t = time))
            self._runtime_arguments['xdot_ref_goal_0_leaf'] = np.array(self._goal.primary_goal().velocity(t = time))
            self._runtime_arguments['xddot_ref_goal_0_leaf'] = np.array(self._goal.primary_goal().acceleration(t = time))
        else:
            self._runtime_arguments['x_goal_0'] = np.array(self._goal.primary_goal().position())
        if len(args) > 2:
            ob_lidar = args[2].reshape(self._config.number_lidar_rays, 2) + args[0][0:2]
            ob_lidar = np.append(ob_lidar, np.zeros((self._config.number_lidar_rays, 1)), axis=1)
        else:
            ob_lidar = [[100, 100, 100],] * self._config.number_lidar_rays
        for j in range(self._config.number_lidar_rays):
            self._runtime_arguments[f'radius_obst_{j}'] = np.array([self._config.radius_ray_obstacles])
            self._runtime_arguments[f'x_obst_{j}'] = ob_lidar[j]

    def computeAction(self, *args):
        self.adapt_runtime_arguments(args)
        action = np.zeros(3)
        action = self._planner.compute_action(**self._runtime_arguments)
        return action

