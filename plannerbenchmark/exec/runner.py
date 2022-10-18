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

from plannerbenchmark.generic.global_planner import GlobalPlannerFoundNoSolution

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
        self._parser.add_argument("--ros", dest="ros", action="store_true")
        self._parser.add_argument("--no-save", dest="save", action="store_false")
        self._parser.add_argument("--random-goal", dest="random_goal", action="store_true")
        self._parser.add_argument("--random-obst", dest="random_obst", action="store_true")
        self._parser.add_argument("--random-init", dest="random_init", action="store_true")
        self._parser.add_argument("--no-verbose", dest="verbose", action="store_false")
        self._parser.add_argument("--render", dest="render", action="store_true")
        self._parser.add_argument("--compare", dest="compare", action="store_true")
        self._parser.add_argument("--global-planning", dest="global_planning", action="store_true")
        self._parser.set_defaults(global_planning=False)
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
        self._global_planning = args.global_planning
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
        self._ros_converter = ActionConverterNode(dt, rate_int, self._experiment.robot_type())

    def setPlanner(self):
        for planner in self._planners:
            planner.reset()
            planner.setJointLimits(self._experiment.limits())
            planner.setSelfCollisionAvoidance(self._experiment.rBody())
            planner.setObstacles(self._experiment.obstacles(), self._experiment.rBody())
            planner.setGoal(self._experiment.goal())
            planner.concretize()

    def step(self, action: np.ndarray, t_exp) -> tuple:
        """Steps the simulation by one time step and applies the action.

        If the gym environment is used, the simulation is discrete in time.
        When the ros bridge is used on the other side, a pseude discrete step
        is used. As the the real world moves on, the action is only applied at
        one moment and the observation belongs to the newest data available at
        that moment.

        Parameters
        -----------

        action: np.ndarray
            action that is send to the robot.


        Returns
        -------------
        observation: dict
            observation dictionary that contains all the information about
            joint states and environment.
        """
        if self._ros:
            self._ros_converter.publish_action(action)
            observation, t = self._ros_converter.observe()
            #self._ros_converter.setGoal(self._experiment.primary_goal(), t=t_exp)
            #for i, obst in enumerate(self._experiment.obstacles()):
            #    self._ros_converter.setObstacle(obst, i, t=t_exp)
        else:
            observation, _, _, _ = self._env.step(action)
            t = self._env.t()
        return observation, t

    def reset(self, q0, q0dot):
        if self._ros:
            ob, t0 = self._ros_converter.observe()
            self._ros_converter.setGoal(self._experiment.primary_goal())
            for i, obst in enumerate(self._experiment.obstacles()):
                self._ros_converter.setObstacle(obst, i)
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
            self._experiment.restore_original_goal()
            self._experiment.shuffle(self._random_obst, self._random_init, self._random_goal)
            try:
                self._experiment.checkFeasibility(checkGoalReachible=False)
                completedRuns += 1
            except ExperimentInfeasible as e:
                logging.warn(f"Case not feasible, {e}")
                continue
            if self._global_planning:
                try:
                    self._experiment.compute_global_path()
                except GlobalPlannerFoundNoSolution as e:
                    logging.warn(f"Could not find global path")
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
                observation = {
                    "joint_state":{
                        "position": np.zeros(self._experiment.n()),
                        "velocity": np.zeros(self._experiment.n()),
                    },
                    "goal": [],
                    "obstacles": [],
                }
                for i in range(self._experiment.T()):
                    if self._aborted:
                        break
                    if i % 1000 == 0:
                        logging.info(f"Timestep : {i}")
                    if 'robot_0' in ob:
                        ob = ob['robot_0']
                    observation['joint_state'] = ob['joint_state']
                    if self._experiment.robot_type() in ['groundRobot', 'boxer', 'albert']:
                        qudot = np.concatenate((ob['joint_state']['forward_velocity'], ob['joint_state']['velocity'][2:]))

                        observation['joint_state']['forward'] = [qudot]
                    env_evaluation = self._experiment.evaluate(t)
                    observation.update(env_evaluation)
                    t_before = time.perf_counter()
                    action = planner.computeAction(observation)
                    if np.isnan(action).any():
                        logging.warn(f"Action computed ignored because of nan value action: {action}")
                        logging.warn(f"Observation in this time step {observation}")
                        action = np.zeros(self._experiment.n())
                    solving_time = time.perf_counter() - t_before
                    logger.add_result_point(t, observation, action, solving_time)
                    ob, t_new = self.step(action, t)
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
