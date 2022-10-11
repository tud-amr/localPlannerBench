import numpy as np

from ompl import base
from ompl import geometric

from MotionPlanningEnv.collisionObstacle import CollisionObstacle

class GlobalPlannerFoundNoSolution(Exception):
    pass

def point_in_collision_with_obstacle(state: base.State, obstacle: CollisionObstacle):
    robot_radius = 0.1
    state_array = np.array([state[i] for i in range(obstacle.dimension())])
    distance = np.linalg.norm(state_array - obstacle.position()) - obstacle.radius() - robot_radius
    return distance < 0


class GlobalPlanner(object):

    def __init__(self, obstacles: list, m: int):
        self._obstacles = obstacles
        self._m = m
        self.initialize_search_space()

    def initialize_search_space(self):
        bounds = base.RealVectorBounds(self._m)
        bounds.setLow(-5)
        bounds.setHigh(5)
        self._space = base.RealVectorStateSpace(self._m)
        self._space.setBounds(bounds)


    def is_state_valid(self, state):
        for obstacle in self._obstacles:
            if point_in_collision_with_obstacle(state, obstacle):
                return False
        return True

    def setup_planning_problem(self, start_array, goal_array):
        self._search_setup = geometric.SimpleSetup(self._space)
        self._search_setup.setStateValidityChecker(base.StateValidityCheckerFn(self.is_state_valid))
        start_state = base.State(self._space)
        goal_state = base.State(self._space)
        for i in range(self._m):
            start_state[i] = start_array[i]
            goal_state[i] = goal_array[i]
        self._search_setup.setStartAndGoalStates(start_state, goal_state)

    def set_planner(self, planner_description: str):
     # set sampler (optional; the default is uniform sampling)
     si = self._search_setup.getSpaceInformation()
  
     # create a planner for the defined space
     if planner_description == 'rrt':
         planner = geometric.RRT(si)
     else:
         planner = geometric.RRT(si)
     self._search_setup.setPlanner(planner)
  

    def solve_planning_problem(self):
        solved = self._search_setup.solve(10.0)
        if solved:
            # try to shorten the path
            self._search_setup.simplifySolution()
            # print the simplified path
            shortest_path = self._search_setup.getSolutionPath()
            shortest_path.interpolate()
            self._control_points = []
            for state_index in range(shortest_path.getStateCount()):
                self._control_points.append([shortest_path.getStates()[state_index][i] for i in range(self._m)])
        else:
            raise GlobalPlannerFoundNoSolution("The global planner did find a solution.")

    def control_points(self):
        return self._control_points

