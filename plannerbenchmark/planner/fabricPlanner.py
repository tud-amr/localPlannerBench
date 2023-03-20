# General dependencies
from dataclasses import dataclass, field
from typing import Dict
import numpy as np
import logging

# Dependencies for this specific planner
from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner
from fabrics.planner.non_holonomic_parameterized_planner import NonHolonomicParameterizedFabricPlanner

# Motion planning scenes
from mpscenes.goals.dynamic_sub_goal import DynamicSubGoal
from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle

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

    def reset(self):
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
        if self._exp.base_type() == 'holonomic':
            self._planner = ParameterizedFabricPlanner(
                self.config.n,
                "useless",
                urdf=self._exp.urdf(),
                root_link=self._exp.root_link(),
                end_link=self._exp.ee_links()[0],
                base_energy=base_energy,
                collision_geometry=collision_geometry,
                collision_finsler=collision_finsler,
                self_collision_geometry=self_collision_geometry,
                self_collision_finsler=self_collision_finsler,
            )
        elif self._exp.base_type() == 'diffdrive':
            collision_geometry = "-2.0 / (x ** 1) * xdot ** 2"
            collision_finsler = "1.0/(x**1) * (1 - ca.heaviside(xdot))* xdot**2"
            self._planner = NonHolonomicParameterizedFabricPlanner(
                self.config.n,
                "boxer",
                collision_geometry=collision_geometry,
                collision_finsler=collision_finsler,
                l_offset="0.1/ca.norm_2(xdot)",
            )
        self._collision_links = self._exp.collision_links()
        self._dynamic_goal = False


    def initialize_runtime_arguments(self):
        self._runtime_arguments = {}
        if self._exp.base_type() == 'diffdrive':
            self._runtime_arguments['m_rot'] = 0.2
            self._runtime_arguments['m_base_x'] = 1.5
            self._runtime_arguments['m_base_y'] = 1.5

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
        self._dynamic_goal = isinstance(goal.primary_goal(), DynamicSubGoal) 
        self._goal = goal

    def concretize(self):
        self._planner.set_components(
            self._collision_links,
            self._self_collision_dict,
            self._goal,
            #limits=self._limits,
            number_obstacles=self._number_static_obstacles,
            number_dynamic_obstacles=self._number_dynamic_obstacles,
        )
        self._planner.concretize()
        self.initialize_runtime_arguments()


    def adapt_runtime_arguments(self, **kwargs):
        self._runtime_arguments['q'] = kwargs['joint_state']['position']
        self._runtime_arguments['qdot'] = kwargs['joint_state']['velocity']
        if self._exp.base_type() == 'diffdrive':
            vel = np.concatenate((kwargs['joint_state']['forward_velocity'] , kwargs['joint_state']['velocity'][2:]))
            self._runtime_arguments['qudot'] = vel
        i = -1
        for _ , obstacle in kwargs['FullSensor']['obstacles'].items():
            i += 1
            self._runtime_arguments[f'radius_obst_{i}'] = obstacle['size']
            self._runtime_arguments[f'x_obst_{i}'] = obstacle['position']
            for collision_link in self._exp.collision_links():
                self._runtime_arguments[f'x_ref_dynamic_obst_{i}_{collision_link}_leaf'] = obstacle['position']
                self._runtime_arguments[f'xdot_ref_dynamic_obst_{i}_{collision_link}_leaf'] = obstacle['velocity']
                self._runtime_arguments[f'xddot_ref_dynamic_obst_{i}_{collision_link}_leaf'] = np.zeros(3)
        for i, sub_goal in enumerate(self._goal.sub_goals()):
            goal_key = list(kwargs['FullSensor']['goals'].keys())[i]
            self._runtime_arguments[f'weight_goal_{i}'] = sub_goal.weight() * self.config.attractor['k_psi']
            self._runtime_arguments[f'x_goal_{i}'] = kwargs['FullSensor']['goals'][goal_key]['position'][sub_goal.indices()]

    def computeAction(self, **kwargs):
        self.adapt_runtime_arguments(**kwargs)
        action = self._planner.compute_action(**self._runtime_arguments)
        if np.any(np.isnan(action)):
            action = np.zeros_like(action)
            logging.warning(f"Action computed by fabric contained nan.")
        if self._exp.control_mode() == 'vel':
            action = kwargs['joint_state']['velocity'] + action * self._exp.dt()
        logging.debug(f"Action computed by fabric: {action}")
        return action
