---
Getting Started
---

This is the guide to quickly get going with the local motion planning
benchmark suite. A detailed documentation can be found on 
https://maxspahn.github.io/localPlannerBench/.

Pre-requisites
==============

-   Linux Ubuntu LTS &gt;= 18.04
-   Python &gt;3.6, &lt; 3.10
-   pip3
-   gnuplot (`sudo apt install gnuplot`)
-   \[Optional\] [poetry](https://python-poetry.org/docs/)
-   \[Optional\] [embotech forces
    pro](https://www.embotech.com/products/forcespro/overview/) for mpc

Installation
============

You first have to download the repository

``` {.sourceCode .bash}
git clone git@github.com:maxspahn/localPlannerBench.git
```

Then, you can install the package using pip as:

``` {.sourceCode .bash}
pip3 install .
```

Optional: Installation with poetry
==================================

If you want to use [poetry](https://python-poetry.org/docs/), you have
to install it first. See their webpage for instructions
[docs](https://python-poetry.org/docs/). Once poetry is installed, you
can install the virtual environment with the following commands. Note
that during the first installation `poetry update` takes up to 300 secs.

``` {.sourceCode .bash}
poetry update
poetry install
```

The virtual environment is entered by

``` {.sourceCode .bash}
poetry shell
```

Tutorial
========

The following is a very simple example case.

Run an experiments:

Experiments should be added in separate folder in `experiments`. One
very simple example can be found in this folder. Note that you need to
active your poetry shell if you have installed the package using poetry
by

``` {.sourceCode .bash}
poetry shell
```

Then you navigate there by

``` {.sourceCode .bash}
cd experiments/1_fabric_mpc/pointMass
```

Then the experiment is run with the command line interface

``` {.sourceCode .bash}
../../../plannerbenchmark/exec/runner -c setup/exp.yaml -p fabric setup/fabric.yaml --render
```

Postprocessing:

The experiments can be postprocessed using the provide executable. When
using poetry, make sure you are in the virtual environment
(`poetry shell`)

``` {.sourceCode .bash}
cd path/to/folder/of/experiment
```

The you can run the post processor with arguments as

``` {.sourceCode .bash}
../../../plannerbenchmark/exec/postProcessor --exp path/to/experiment -k time2Goal pathLength --plot
```

More detailed information about this example can be found in
example\_long
