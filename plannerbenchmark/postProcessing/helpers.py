from plannerbenchmark.postProcessing.metrics import (
    SolverTimesMetric,
    ClearanceMetric,
    InverseClearanceMetric,
    TimeToReachGoalMetric,
    PathLengthMetric,
    SuccessMetric,
    IntegratedErrorMetric,
    DynamicClearanceMetric,
    InverseDynamicClearanceMetric,
)
from plannerbenchmark.generic.experiment import Experiment

indexMap = {0: "x", 1: "y", 2: "z"}


def createMetricsFromNames(
    names: str, experiment: Experiment, interval: int = 1
) -> list:
    """Create metrics from the names.

    For every metric different information of the experiment is needed. This
    function extracts the right information of the experiment and the planner
    to form the metrics based on their names.

    Parameters
    ----------
    names : str
        metric names
    experiment : Experiment
        Experiment instance for which the metrics should be added.
    interval : int
        Interval of the planner. This is needed for the solverTime metric.
        (by default it is set to 1, indicating that the planner was
        executed at every time step)

    Returns
    -------
    list
        Returns a list of all metrics for which the name was specified.
    """
    metrics = []
    goal_indices = experiment.primeGoal().indices()
    dimension_obstacle = 3
    n = experiment.n()
    fksNames = []
    fks_collision_link_names = []
    obstacle_names = []
    eeNames = []
    goalNames = []
    r_obsts = []
    for i_obstacle, _ in enumerate(experiment.obstacles()):
        obstacle_names.append(f'obst_{i_obstacle}_radius')
        for i_dimension in range(3):
            obstacle_names.append(f'obst_{i_obstacle}_{i_dimension}_0')
    for obst in experiment.obstacles():
        if obst.type() != 'sphereObstacle':
            r_obsts.append(obst.radius())
    for link_name in experiment._fk.robot.link_names():
        for j in range(3):
            fksNames.append("fk" + link_name + "_" + indexMap[j])
    for collision_link in experiment.collision_links():
        for j in range(3):
            fks_collision_link_names.append("fk" + collision_link + "_" + indexMap[j])
    for j in goal_indices:
        eeNames.append("fk" + experiment.primeGoal().child_link() + "_" + indexMap[j])
        goalNames.append("goal_0_" + str(j) + "_0")

    for name in names:
        if name == "solverTime":
            metrics.append(
                SolverTimesMetric(name, ["t_planning"], {"interval": interval})
            )
        if name == "clearance":
            metrics.append(
                ClearanceMetric(
                    name,
                    fks_collision_link_names + obstacle_names,
                    {
                        "obstacles": experiment.obstacles(),
                        "number_obstacles": int(len(obstacle_names)/4),
                        "number_collision_links": int(len(fks_collision_link_names)/3),
                        "r_body": experiment.rBody(),
                    },
                )
            )
        if name == "invClearance":
            metrics.append(
                InverseClearanceMetric(
                    name,
                    fks_collision_link_names + obstacle_names,
                    {
                        "obstacles": experiment.obstacles(),
                        "number_obstacles": int(len(obstacle_names)/4),
                        "number_collision_links": int(len(fks_collision_link_names)/3),
                        "r_body": experiment.rBody(),
                    },
                )
            )
        if name == "invDynamicClearance":
            metrics.append(
                InverseDynamicClearanceMetric(
                    "clearance", 
                    fks_collision_link_names + ['t'],
                    {
                        'r_body': experiment.rBody(),
                        'r_obsts': r_obsts, 
                        'dimension_obstacle': dimension_obstacle,
                        "collision_links": experiment.collision_links(),
                    }
                )
            )
        if name == "dynamicClearance":
            metrics.append(
                DynamicClearanceMetric(
                    "clearance", 
                    fks_collision_link_names + ['t'],
                    {
                        'r_body': experiment.rBody(),
                        'r_obsts': r_obsts, 
                        'dimension_obstacle': dimension_obstacle,
                    }
                )
            )
        if name == "time2Goal":
            metrics.append(
                TimeToReachGoalMetric(
                    name,
                    eeNames + goalNames + ["t"],
                    {"goal_indices": goal_indices, "des_distance": experiment.primeGoal().epsilon()},
                )
            )
        if name == "pathLength":
            metrics.append(PathLengthMetric(name, eeNames, {}))
        if name == "integratedError":
            metrics.append(
                IntegratedErrorMetric(
                    name,
                    eeNames + goalNames + ["t"],
                    {"dimension_obstacle": dimension_obstacle, "des_distance": 10* experiment.primeGoal().epsilon()},
                )
            )
    return metrics
