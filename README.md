# Experiments for paper

All experiments related to the dynamic fabrics paper should be here.
This repository depends on the dynamic fabrics library (and not much else!).

## How to run the experiments

# Single runs

```bash
cd path/to/experiment-folder
./run_experiment -case setup/<setup-file> -mpc setup/<mpc-file> -fab setup/<fab-file>
```
Postprocessing
```bash
cd postProcessing
./postProcessor path/to/result-folder
```
Results will be displayed and added to the result folder of the experiment as markdown
Plotting trajectories and solvertimes 
```bash
cd postProcessing
cd plotting<robottype>
./createPlots <ndof> path/to/mpc-result <intervals>
```

# running a series
```bash
cd path/to/experiment-folder
./run_series results/<series-name>
```
Postprocessing
```bash
cd postProcessing
./postProcessor path/to/result-folder/<all-files>
```
Postprocessing results will only be added to the results folder; there are no direct
printouts.
Plotting trajectories can be done for individual paths.


## ToDo's
Beside the obvious, some things need to be changed/added?
* [x] Fabrics tend to produce 'nan' values if they are close to the goal for a long time.
  - [x] solved by correcting small values to zeros in fabric planner
* [x] Self-collision avoidance
  - [x] done for fabrics by introducing additional geometries
  - [x] done for mpc
* [x] Postprocessing for
  - [x] path length
  - [x] clearance
* [x] Multi test script
* [x] Dynamic degrees of freedom during post processing
  - [x] Done by adding argument to createPlot script
* [x] Joint limits for fabrics (needed to have a fair comparision)
  - [x] Done according to paper "Optimization fabrics"
* [x] Variable joint limits for mpc
  - [x] Done using additional inequality constraints (consider using different slack
    variabeles)
* [ ] Issue when starting really close to obstacles with fabrics
  -  Note: the issue is really: What happens if a constraints is violated at some point?
* [x] Correction self collision with mpc for planar robots (body 0 must be avoided)
* [x] Add self collision clearenc to clearenc metric

## Metrics

Local motion planning are assessed based on
* [ ] Planning Time: How long does it take to solve the local planning problem?
  - [x] mean solving time (`SolverTimesMetric`)
  - How to process this?
* [ ] Success Rate: Which percantage of cases lead to reaching the goal?
  - [x] Evaluate whether the goal is reached (without colliding) (`SuccessMetric`)
  - Note that this must be done on a series of experiments
* [ ] Path Length
  - [ ] How long is the path taken?
    - [ ] in configuration space (may not make sense as the configuration is not unique?)
    - [x] in end-effector space (`PathLengthMetric`)
  - [x] How much time does it take to reach the goal? (`TimeToReachgoalMetric`)
* [ ] Clearance: 
  - [x] How far away from obstacles does the robot stay? (`ClearanceMetric`)
  - [ ] How fast does it move close to them?
  - [ ] How far from joint limits?
* [x] Integrated Error:
  - [x] How big is the accumlated error once a trajectory goal is reached? (`IntegratedErrorMetric`)

* (Smoothness): ???

## Overview Experements and robots

| Expermient | PointMass | PlanarArm | Panda | RealPanda | Albert |
| -----------|---------|---------|-----|---------|------|
| mpc vs fabric | [x] | [x] | [x] | [ ] | [ ]|
| static vs dynamic| [x] | [ ] | [ ] | [ ] | [ ]|
| moving obstacles | [ ] | [ ] | [ ] | [ ] | [ ]|

## Experiment 1: Fabric vs MPC

The first experiment should highlight the computational advantage of fabrics over model
predictive control.

### a) point mass
<p align="left">
  <img src="./assets/1_comparison_mpc/pointMass/fabric_trajectory-1.png" width="450" title="hover text">
</p>

### b) planar robotic arm(s)
<p align="left">
  <img src="./assets/1_comparison_mpc/planarArm/fabric_trajectory-1.png" width="450" title="hover text">
</p>
<p align="left">
  <img src="./assets/1_comparison_mpc/planarArm/fabric_trajectory2-1.png" width="450" title="hover text">
</p>

### c) real robotic arm
<p align="left">
  <img src="./assets/1_comparison_mpc/simPanda/fabric_trajectory.gif" width="450" title="hover text">
</p>

### d) real mobile manipulator(s)


## Experiment 2: Static Fabric vs Dynamic Fabric

Proof the practical need for dynamic fabrics as they integrate global planning

## Experiment 3: Moving obstacles (Dynamic Fabric vs MPC)

## Experiment 4: Application to (non-)holonomc bases

## Experiment 5: Application to multi-arm systems
