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
    dimension_obstacle = experiment.obstacles()[0].dim()
    n = experiment.n()
    fksNames = []
    eeNames = []
    goalNames = []
    r_obsts = []
    for obst in experiment.obstacles():
        if obst.type() != 'sphereObstacle':
            r_obsts.append(obst.radius())
    for i in range(1, n + 1):
        for j in goal_indices:
            fksNames.append("fk" + str(i) + "_" + indexMap[j])
    for j in goal_indices:
        eeNames.append("fk" + str(n) + "_" + indexMap[j])
        goalNames.append("goal_" + str(j) + "_0")
    for name in names:
        if name == "solverTime":
            metrics.append(
                SolverTimesMetric(name, ["t_planning"], {"interval": interval})
            )
        if name == "clearance":
            metrics.append(
                ClearanceMetric(
                    name,
                    fksNames,
                    {
                        "obstacles": experiment.obstacles(),
                        "n": experiment.n(),
                        "r_body": experiment.rBody(),
                    },
                )
            )
        if name == "invClearance":
            metrics.append(
                InverseClearanceMetric(
                    name,
                    fksNames,
                    {
                        "obstacles": experiment.obstacles(),
                        "n": experiment.n(),
                        "r_body": experiment.rBody(),
                    },
                )
            )
        if name == "invDynamicClearance":
            metrics.append(
                InverseDynamicClearanceMetric(
                    "clearance", 
                    fksNames + ['t'],
                    {
                        'r_body': experiment.rBody(),
                        'r_obsts': r_obsts, 
                        'dimension_obstacle': dimension_obstacle,
                        'n': experiment.n()
                    }
                )
            )
        if name == "dynamicClearance":
            metrics.append(
                DynamicClearanceMetric(
                    "clearance", 
                    fksNames + ['t'],
                    {
                        'r_body': experiment.rBody(),
                        'r_obsts': r_obsts, 
                        'dimension_obstacle': dimension_obstacle,
                        'n': experiment.n()
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
