.. _example_long:

Example
==================

We provide several experiment setups in the ``experiments`` folder.
However, here we want to walk you through the most basic example without a planner that
has a dependency on other packages.

Navigate to example
-------------------
First, we must ensure that everything is installed and enter the virtual environment.

.. code:: bash

   cd /path/to/localPlannerBench
   poetry install
   poetry shell
   cd experiments/example

Nice! In this folder, you should see a ``setup`` directory and a ``results`` directory.
The first contains the configuration files for the planner ``planner.yaml`` and 
the experiment ``exp.yaml``. Go ahead and look a bit for yourself where the obstacles,
goals and parameters to the planner are specified.

Run the example
----------------

Once you have familiarized yourself with the directories and files, you can run the
example. For that execute

.. code:: bash

  runner -c setup/exp.yaml -p setup/planner.yaml --render

When you run the above line, the experiment starts with the configuration files you have
provided through the command line interface, namely ``-c setup/exp.yaml`` specifies the
experiment and ``-p setup/planner.yaml`` specifies the planner. The flag
``--render`` indicates that the experiment should be rendered to your screen. Once the
experiment has finished or you have stopped it prematurely, a new folder is created in the
``results`` directory. In there, you will find all necessary information on what exactly
happened during the expereiment and all the configuration you have set.

Postprocessing
---------------

In the previous step, you have run the experiment and you now want to access the
performance of the particular planner. This is what the ``postProcessor`` is all about. 
The postprocessor uses the results folder that was created after running the experiment to
evaluate user-specified metrics. Familiarize yourself a bit with the content of the folder
you want to postprocess. When you are ready, you can simple invoke the postProcessor by 

.. code:: bash

   post_process --exp results/<name_experiment> -k time2Goal pathLength --plot

In the above line, the argument ``--exp results/<name_experiment>`` tells the
postProcessor which experiment to process. Key-performance-indicators are listed behind
``-k`` option. The flag ``--plot`` that a plot of the trajectory should be created.
Navigate to the experiment folder in your explorer to
access the plots and evaluations.
The evaluations are stored in ``postProcess.yaml``.


Conclusion
------------

Nice! You have run your first experiment for testing a local motion planning algorithm.
Feel free to add your own method or additional functionility.

