#! /usr/bin/env python3

import subprocess
import argparse
import datetime
import time
import os
import numpy as np
import signal
import logging
import hydra
from dataclasses import dataclass 
from hydra.core.config_store import ConfigStore
from omegaconf import DictConfig, MISSING
from typing import Optional

from plannerbenchmark.generic.experiment import Experiment, ExperimentInfeasible, ExperimentConfig
from plannerbenchmark.generic.logger import Logger

from plannerbenchmark.generic.planner  import PlannerRegistry, PlannerConfig
from plannerbenchmark.planner import *

log_levels = {"WARNING": 30, "INFO": 20, "DEBUG": 10, "QUIET": 100}


@dataclass
class RunnerConfig:
    """Class comment to be filled in"""

    render: bool = True
    save: bool = True
    ros: bool = False
    config_folder: Optional[str] = None
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
cs.store(name="base_runner", node=RunnerConfig)
# print(cs.repo.keys())


class Runner(object):
    def __init__(self):
        signal.signal(signal.SIGINT, self.handleStop)
        self._aborted = False

    def handleStop(self, signum, frame):
        logging.info("\n Stopping runner...")
        self.stopEnvironment()
        self._aborted = True

    def parseArguments(self, cfg: DictConfig):
        self._render = cfg.render
        self._save = cfg.save
        self._random_obst = cfg.random_obst
        self._random_init = cfg.random_init
        self._random_goal = cfg.random_goal
        self._numberRuns = cfg.numberRuns
        self._verbose = cfg.verbose
        self._render = cfg.render
        self._res_folder = os.getcwd() + '/' + cfg.res_folder
        self._planners = []
        self._experiment = Experiment(cfg.experiment)
        self._ros = cfg.ros
        self._compare = cfg.compare
        if self._compare:
            self._compareTimeStamp = 0
        if self._ros:
            self.startRosConverterNode()
        else:
            self._env = self._experiment.env(render=self._render)

        # Only supports one planner config at a time with hydra sturctured configs,
        # because list or dicts in the default are not supported.
        # There is an easy api for running multiple configurations in a sweep though. 
        planner = PlannerRegistry.create_planner(self._experiment, cfg.planner)
        self._planners.append(planner)

    def startRosConverterNode(self):
        from plannerbenchmark.ros.ros_converter_node import ActionConverterNode
        dt = self._experiment.dt()
        rate_int = int(1/dt)
        self._rosConverter = ActionConverterNode(dt, rate_int, self._experiment.robotType())

    def setPlanner(self):
        for planner in self._planners:
            planner.reset()
            planner.setJointLimits(self._experiment.limits())
            planner.setSelfCollisionAvoidance(self._experiment.rBody())
            planner.setObstacles(self._experiment.obstacles(), self._experiment.rBody())
            planner.setGoal(self._experiment.goal())
            planner.concretize()

    def applyAction(self, action, t_exp):
        if self._ros:
            ob, t = self._rosConverter.publishAction(action)
            self._rosConverter.setGoal(self._experiment.primeGoal(), t=t_exp)
            for i, obst in enumerate(self._experiment.obstacles()):
                self._rosConverter.setObstacle(obst, i, t=t_exp)
        else:
            ob, _, _, _ = self._env.step(action)
            t = t_exp + self._experiment.dt()
        return ob, t

    def reset(self, q0, q0dot):
        if self._ros:
            ob, t0 = self._rosConverter.ob()
            self._rosConverter.setGoal(self._experiment.primeGoal())
            for i, obst in enumerate(self._experiment.obstacles()):
                self._rosConverter.setObstacle(obst, i)
        else:
            ob = self._env.reset(pos=q0, vel=q0dot)
            t0 = 0.0
        return ob, t0

    def stopEnvironment(self):
        """
        if self._ros:
            res = subprocess.run(['/bin/zsh', '-i', '-c', 'stopMotion'], stdout=subprocess.PIPE)
            print(res)
        """
        return

    def run(self):
        logging.info("Starting runner...")
        completedRuns = 0
        while completedRuns < self._numberRuns:
            self._experiment.shuffle(self._random_obst, self._random_init, self._random_goal)
            try:
                self._experiment.checkFeasibility(checkGoalReachible=False)
                completedRuns += 1
            except ExperimentInfeasible as e:
                logging.warn(f"Case not feasible, {e}")
                continue
            logging.info("Composing the planner")
            start=time.perf_counter()
            self.setPlanner()
            logging.info(f"Planner composed in {np.round(time.perf_counter()-start, decimals=2)} sec")
            q0, q0dot = self._experiment.initState()
            timeStamp = "{:%Y%m%d_%H%M%S_%f}".format(datetime.datetime.now())
            if self._compare:
                timeStamp = self._compareTimeStamp
            for planner in self._planners:
                ob, t0 = self.reset(q0, q0dot)
                if not self._ros:
                    self._experiment.addScene(self._env)
                logger = Logger(self._res_folder, timeStamp)
                logger.setSetups(self._experiment, planner)
                t = 0.0
                for i in range(self._experiment.T()):
                    if self._aborted:
                        break
                    if i % 1000 == 0:
                        logging.info(f"Timestep : {i}")
                    if 'x' in ob:
                        q = ob['x']
                        qdot = ob['xdot']
                    elif 'joint_state' in ob:
                        q = ob['joint_state']['position']
                        qdot = ob['joint_state']['velocity']
                    if self._experiment.dynamic():
                        envEval = self._experiment.evaluate(t)
                        if not planner.config.dynamic:
                            envEval[1] = np.zeros(envEval[1].size)
                            envEval[2] = np.zeros(envEval[2].size)
                        observation = [q, qdot] + envEval
                    else:
                        observation = [q, qdot]
                    if self._experiment.robotType() in ['groundRobot', 'boxer', 'albert']:
                        qudot = np.concatenate((ob['joint_state']['forward_velocity'], ob['joint_state']['velocity'][2:]))
                        observation += [qudot]
                    t_before = time.perf_counter()
                    action = planner.computeAction(*observation)
                    solving_time = time.perf_counter() - t_before
                    primeGoal = [self._experiment.evaluatePrimeGoal(t)]
                    obsts = self._experiment.evaluateObstacles(t)
                    obsts_cleaned = [obsts[i:i+3] for i in range(0, len(obsts), 3)]
                    logger.addResultPoint(t, q, qdot, action, solving_time, primeGoal, obsts_cleaned)
                    ob, t_new = self.applyAction(action, t)
                    t = t_new - t0
                if self._save:
                    logger.save()
            logging.info(f"Completed {completedRuns} runs")
            self.stopEnvironment()

@hydra.main(version_base=None, config_path="../../examples/point_robot", config_name="config")
def main(cfg):
    myRunner = Runner()
    myRunner.parseArguments(cfg)
    myRunner.run()


if __name__ == "__main__":
    main()
