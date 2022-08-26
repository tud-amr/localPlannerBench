.. _experiment_config:

Experiment Config
==================

In the example you have learned how easy it is to run different experiment setups in combination with different planners. 
LocalPlannerBench is especially powerful because of the versatility of the supported experiment options and environments. 
To create your own custom experiment setup, simply write an ``exp.yaml`` file with your desired configurations.
We will list the available options here:

:T :int:
    Maximum simulated time to let the experiment run for. 

:dt :float:
    Timestep of the simulator.

:dynamic :bool:
    Does the experiment contain dynamic obstacles or goals.

:env :str:
    Name of the simulation environment. LocalPlannerBench currently supports 2 different simulators with a large number of environments and robots of varying fidelity from point mass robots to full fledged mobile manipulators. 
    More information about the two simulators can be found here:

    * `gym_envs_planar <https://github.com/maxspahn/gym_envs_planar>`_
    * `gym_envs_urdf <https://github.com/maxspahn/gym_envs_urdf>`_

:goal :Dict:
    Dictionary containing a collection of subgoals for the current experiment. The keys of the subgoals should follow this structure: ``subgoal0, subgoal1, ..., subgoali``.
    A subgoal specification is again very versatile, you can specify the indices of the joints, desired position, weight of the goal and even minimum and maximum goal values for random generation of goals.  
    See `motion_planning_scenes <https://github.com/maxspahn/motion_planning_scenes>`_ for more information.

:initState :Dict:
    Should contain a dictionary with the initial state of the joints. The key ``q0`` containing the initial positions and ``q0dot`` the initial velocities. 

:limits :Dict:
    Should contain a dictionary with the lower and upper limits of the states of the joints. The key ``low`` containing the lower bound positions and ``high`` the upper bound positions. 

:n :int:
    Number of joints

:obstacles :Dict:
    Dictionary containing a collection of obstacles for the current experiment. The keys of the obstacles should follow this structure: ``obst0, obst1, ..., obsti``.
    A subgoal specification is again very versatile, you can specify the dimension, desired position, and even minimum and maximum values for random generation of obstacles.  
    See `motion_planning_scenes <https://github.com/maxspahn/motion_planning_scenes>`_ for more information.

:r_body :float:
    Radius of the links of the robot, to be used by planners for collision avoidance.
    Note: only a single radius for all links is currently supported.

:randomObstacles :Dict:
    Dictionary containing information about the random generation of obstacles for this experiment.
    The Dictionary should contain the key ``number`` to specify the number of randomobstacles.

:robot_type :str:
    The name of the type of robot used in the environment.

:selfCollision :Dict:
    Dictionary containing information about configuration for self collision avoidance.
    The Dictionary should contain the key ``pairs`` to specify pairs of links that should avoid eachother.

