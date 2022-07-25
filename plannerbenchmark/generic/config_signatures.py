from dataclasses import dataclass
from omegaconf import DictConfig, OmegaConf, MISSING
import hydra
from hydra.core.config_store import ConfigStore
from typing import List, Dict, Union, Any, Optional

@dataclass
class LocalPlannerConfig:
    """Data class that specifies the structure and default values of a planner config. 
    Note: this is a base class that should be implemented when making your own custom planner"""

    #: Comment to be filled in 
    name: str = MISSING

# Add your own planners derived from the LocalPlannerConfig base
@dataclass
class PdConfig(LocalPlannerConfig):
    """Class comment to be filled in"""

    #: Comment to be filled in 
    robot_type: str = MISSING

    #: Comment to be filled in 
    n: int = MISSING

    #: Comment to be filled in 
    k: float = MISSING

    #: Comment to be filled in 
    p: float = MISSING

@dataclass
class MpcConfig(LocalPlannerConfig):
    """Class comment to be filled in"""

    #: Comment to be filled in 
    robot_type: str = MISSING

    #: Comment to be filled in 
    n: float = MISSING

    #: Comment to be filled in 
    k: float = MISSING

    #: Comment to be filled in 
    p: float = MISSING

@dataclass
class FabricsConfig(LocalPlannerConfig):
    """Class comment to be filled in"""

    #: Comment to be filled in 
    interval: int = MISSING

    #: Comment to be filled in 
    robot_type: str = MISSING

    #: Comment to be filled in 
    n: float = MISSING

    #: Comment to be filled in 
    m_base: float = MISSING

    #: Comment to be filled in 
    obst: Dict[str, float] = MISSING

    #: Comment to be filled in 
    selfCol: Dict[str, float] = MISSING

    #: Comment to be filled in 
    attractor: Dict[str, float] = MISSING

    #: Comment to be filled in 
    limits: Dict[str, float] = MISSING

    #: Comment to be filled in 
    speed: Dict[str, float] = MISSING

    #: Comment to be filled in 
    damper: Dict[str, float] = MISSING

    #: Comment to be filled in 
    dynamic: bool = MISSING

@dataclass
class SubGoalCompositionConfig:
    """Class comment to be filled in"""

    #: Comment to be filled in 
    prime: bool = MISSING

    #: Comment to be filled in 
    m: float = MISSING

    #: Comment to be filled in 
    w: float = MISSING

    #: Comment to be filled in 
    indices: List[int] = MISSING

    #: Comment to be filled in 
    parent_link: int = MISSING

    #: Comment to be filled in 
    child_link: int = MISSING

    #: Comment to be filled in 
    desired_position: List[float] = MISSING

    #: Comment to be filled in 
    low: List[float] = MISSING

    #: Comment to be filled in 
    high: List[float] = MISSING

    #: Comment to be filled in 
    type: str = MISSING

    #: Comment to be filled in 
    epsilon: float = MISSING

@dataclass
class StateConfig:
    """Class comment to be filled in"""

    #: Comment to be filled in 
    q0: List[float] = MISSING

    #: Comment to be filled in 
    q0dot: List[float] = MISSING

@dataclass
class LimitsConfig:
    """Class comment to be filled in"""

    #: Comment to be filled in 
    low: List[float] = MISSING

    #: Comment to be filled in 
    high: List[float] = MISSING

@dataclass
class RandomObstaclesConfig:
    """Class comment to be filled in"""

    #: Comment to be filled in 
    number: int = MISSING

@dataclass
class GeometryConfig:
    """Class comment to be filled in"""

    #: Comment to be filled in 
    position: List[float] = MISSING

    #: Comment to be filled in 
    radius: float = MISSING

@dataclass
class ObstacleConfig:
    """Class comment to be filled in"""

    #: Comment to be filled in 
    type: str = MISSING

    #: Comment to be filled in 
    dim: int = MISSING

    #: Comment to be filled in 
    geometry: GeometryConfig = MISSING

    #: Comment to be filled in 
    low: GeometryConfig = MISSING

    #: Comment to be filled in 
    high: GeometryConfig = MISSING

@dataclass
class SelfCollisionConfig:
    """Class comment to be filled in"""

    #: Comment to be filled in 
    pairs: Optional[List[List[int]]] = MISSING

@dataclass
class ExperimentConfig:
    """Data class that specifies the structure and default values of an experiment config"""

    #: Time I think?
    T: int = 2000 

    #: Number of something?
    n: int = MISSING

    #: Dynamic something?
    dynamic: bool = MISSING

    #: Timestep for the simulator?
    dt: float = 0.05 

    #: Environment to use in the simulator?
    env: str = MISSING

    #: Comment to be filled in 
    robot_type: str = MISSING

    #: Comment to be filled in 
    goal: Dict[str, SubGoalCompositionConfig] = MISSING

    #: Comment to be filled in 
    initState: StateConfig = MISSING

    #: Comment to be filled in 
    limits: LimitsConfig = MISSING

    #: Comment to be filled in 
    r_body: float = MISSING

    #: Comment to be filled in 
    randomObstacles: RandomObstaclesConfig = MISSING

    #: Comment to be filled in 
    obstacles: Dict[str, ObstacleConfig] = MISSING

    #: Comment to be filled in 
    selfCollision: SelfCollisionConfig = MISSING

@dataclass
class LocalPlannerBenchConfig:
    """Class comment to be filled in"""

    #: Comment to be filled in 
    render: bool = True

    #: Comment to be filled in 
    save: bool = True

    #: Comment to be filled in 
    ros: bool = False

    #: Comment to be filled in 
    res_folder: str = 'results'

    #: Comment to be filled in 
    random_goal: bool = False

    #: Comment to be filled in 
    random_obst: bool = False

    #: Comment to be filled in 
    random_init: bool = False

    #: Comment to be filled in 
    verbose: bool = False

    #: Comment to be filled in 
    compare: bool = False

    #: Comment to be filled in 
    numberRuns: int = 1

    #: Comment to be filled in 
    planner: LocalPlannerConfig = MISSING # no static type checking for planners because Unions of containers are not supported

    #: Comment to be filled in 
    experiment: ExperimentConfig = MISSING

cs = ConfigStore.instance()
cs.store(name="base_config", node=LocalPlannerBenchConfig)
cs.store(group="experiment", name="base_experiment", node=ExperimentConfig)
cs.store(group="planner", name="base_pd", node=PdConfig)
cs.store(group="planner", name="base_fabric", node=FabricsConfig)

# @hydra.main(version_base=None, config_path=".", config_name="config")
# def my_app(cfg: DictConfig) -> None:
#     print(OmegaConf.to_yaml(cfg))

# if __name__ == "__main__":
#     my_app()
