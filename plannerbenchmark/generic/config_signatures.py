from dataclasses import dataclass
from omegaconf import MISSING
from hydra.core.config_store import ConfigStore

from plannerbenchmark.generic.planner import PlannerConfig
from plannerbenchmark.generic.experiment import ExperimentConfig
from plannerbenchmark.planner.pdPlanner import PdConfig
from plannerbenchmark.planner.acadosMpcPlanner import AcadosMpcConfig

# NOTE: This file is only needed to register the default class signatures of the available planners, so hydra knows about them.

@dataclass
class LocalPlannerBenchConfig:
    """Class comment to be filled in"""

    render: bool = True
    save: bool = True
    ros: bool = False
    res_folder: str = 'results'
    random_goal: bool = False
    random_obst: bool = False
    random_init: bool = False
    verbose: bool = False
    compare: bool = False
    numberRuns: int = 1
    planner: PlannerConfig = MISSING # no static type checking for planners because Unions of containers are not supported
    experiment: ExperimentConfig = MISSING

cs = ConfigStore.instance()
cs.store(name="base_config", node=LocalPlannerBenchConfig)
cs.store(group="experiment", name="base_experiment", node=ExperimentConfig)
cs.store(group="planner", name="base_pd", node=PdConfig)
cs.store(group="planner", name="base_acados_mpc", node=AcadosMpcConfig)
