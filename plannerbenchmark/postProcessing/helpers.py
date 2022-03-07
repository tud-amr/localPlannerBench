from plannerbenchmark.postProcessing.metrics import SolverTimesMetric, ClearanceMetric, TimeToReachGoalMetric, PathLengthMetric, SuccessMetric

indexMap = {0: 'x', 1: 'y', 2: 'z'}


def createMetricsFromNames(names, experiment, interval=1):
    metrics = []
    m = experiment.obstacles()[0].dim()
    n = experiment.n()
    fksNames = []
    eeNames = []
    goalNames = []
    for i in range(1, n + 1):
        for j in range(m):
            fksNames.append("fk" + str(i) + "_" + indexMap[j])
    for j in range(m):
        eeNames.append("fk" + str(n) + "_" + indexMap[j])
        goalNames.append("goal_" + str(j) + "_0")
    for name in names:
        if name == 'solverTime':
            metrics.append(
                SolverTimesMetric(name, ["t_planning"], {"interval": interval})
            )
        if name == 'clearance':
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
        if name == 'time2Goal':
            metrics.append(
                TimeToReachGoalMetric(
                    name,
                    eeNames + goalNames + ["t"],
                    {"m": m, "des_distance": experiment.primeGoal().epsilon()},
                )
            )
        if name == "pathLength":
            metrics.append(
                PathLengthMetric(
                    name,
                    eeNames,
                    {}
                )
            )
    return metrics
