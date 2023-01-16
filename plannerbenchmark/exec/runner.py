#! /usr/bin/env python3

import argparse
import datetime
import time
import os
import numpy as np
import signal
import logging

from plannerbenchmark.generic.experiment import Experiment, ExperimentInfeasible
from plannerbenchmark.generic.logger import Logger

from plannerbenchmark.generic.planner  import PlannerRegistry
from plannerbenchmark.generic.utils import import_custom_planners
import_custom_planners()
import plannerbenchmark.planner


log_levels = {"WARNING": 30, "INFO": 20, "DEBUG": 10, "QUIET": 100}


class Runner(object):
    def __init__(self):
        self.initializeArgumentParser()
        signal.signal(signal.SIGINT, self.handleStop)
        self._aborted = False

    def handleStop(self, signum, frame):
        logging.info("\n Stopping runner...")
        self.stopEnvironment()
        self._aborted = True

    def initializeArgumentParser(self):
        self._parser = argparse.ArgumentParser("Run motion planning experiment")
        self._parser.add_argument(
            "--caseSetup", "-case", type=str, help="Experiment setup file", required=True
        )
        self._parser.add_argument(
            "--planners", "-p", type=str, nargs="+", help="List of planner setup files", required=True
        )
        self._parser.add_argument(
            "--res-folder",
            "-res",
            type=str,
            default='results',
            help="Results folder",
        )
        self._parser.add_argument(
                "--log-level",
                "-v",
                "-ll",
                type=str,
                default="INFO",
                help="Set logging level (Choose between DEBUG, INFO, WARNING, QUIET)")
        self._parser.add_argument(
            "--numberRuns", "-n", type=int, default=1, help="Number of runs of the experiment"
        )
        self._parser.add_argument(
            "--show-rays", "-sr", type=bool, default=False, help="Show LiDAR ray visualization by setting to True"
        )
        self._parser.add_argument("--ros", dest="ros", action="store_true")
        self._parser.add_argument("--no-save", dest="save", action="store_false")
        self._parser.add_argument("--random-goal", dest="random_goal", action="store_true")
        self._parser.add_argument("--random-obst", dest="random_obst", action="store_true")
        self._parser.add_argument("--random-init", dest="random_init", action="store_true")
        self._parser.add_argument("--no-verbose", dest="verbose", action="store_false")
        self._parser.add_argument("--render", dest="render", action="store_true")
        self._parser.add_argument("--compare", dest="compare", action="store_true")
        self._parser.set_defaults(save=True)
        self._parser.set_defaults(ros=False)
        self._parser.set_defaults(random_goal=False)
        self._parser.set_defaults(random_obst=False)
        self._parser.set_defaults(random_init=False)
        self._parser.set_defaults(verbose=True)
        self._parser.set_defaults(render=False)
        self._parser.set_defaults(compare=False)

    def parseArguments(self):
        args = self._parser.parse_args()
        self._render = args.render
        self._save = args.save
        self._random_obst = args.random_obst
        self._random_init = args.random_init
        self._random_goal = args.random_goal
        self._numberRuns = args.numberRuns
        self._verbose = args.verbose
        logging.basicConfig()
        logging.getLogger().setLevel(log_levels[args.log_level])
        self._render = args.render
        self._res_folder = os.getcwd() + '/' + args.res_folder
        self._planners = []
        self._experiment = Experiment(args.caseSetup)

        # NOTE: Shuffle at least once to make sure experiment.obstacles() has right shape. Important for mpc problem formulation in __init__ functions.
        self._experiment.shuffle(self._random_obst, self._random_init, self._random_goal)

        self._ros = args.ros
        self._compare = args.compare
        self._show_rays = args.show_rays
        if self._compare:
            self._compareTimeStamp = args.caseSetup[-24:-9]
        if self._ros:
            self.startRosConverterNode()
        else:
            self._env = self._experiment.env(render=self._render)
        for planner_file in args.planners:
            planner = PlannerRegistry.create_planner(self._experiment, planner_file)
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
                try:
                    # Sets the LiDAR data to 100.0 for the first step.
                    ob['lidarSensor'] = np.full([planner._config.number_lidar_rays*2,], 100.0)
                except AttributeError:
                    print("No LiDAR sensor available to set default values to 100.0")
                if not self._ros:
                    try:
                        self._experiment.addScene(self._env, planner._config.number_lidar_rays)
                    except AttributeError:
                        print("No LiDAR sensor found, using default environment.")
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
                    try:
                        sensor_data = ob['lidarSensor']
                        if self._show_rays:
                            if i == 0:
                                body_ids_old = None
                            show_lidar_mode = "spheres"
                            show_lidar_step = 1
                            if i in range(1, self._experiment.T(), show_lidar_step):
                                body_ids_old = self._experiment.showLidar(self._env, sensor_data, q, body_ids_old, planner._config.number_lidar_rays, show_lidar_mode)
                        observation += [sensor_data]
                    except KeyError as key_error:
                        observation += [0]
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

def main():
    myRunner = Runner()
    myRunner.parseArguments()
    myRunner.run()


if __name__ == "__main__":
    main()
