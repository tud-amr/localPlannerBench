# Local Motion Planning Benchmark Suite

This repository is meant to allow quick comparison between different local motion planning
algorithms. Running and postprocessing is available and we aim to offer a nice interface
to implement a wrapper to your own motion planner.

## Installation

You first have to download the repository
```bash
git clone git@github.com:maxspahn/localPlannerBench.git
```

The package is setup with poetry. That means you can install it globally using pip
```bash
pip3 install .
```

If you want to use (https://python-poetry.org/docs/)[poetry], you have to install it
first. See their webpage for instruction https://python-poetry.org/docs/.
Once poetry  is installed, you can install the virtual environment with
```bash
poetry update
poetry install
```

The virtual environment is entered by
```bash
poetry shell
```

## Run an experiments

Experiments should be added in separate folder in `experiments`.
Some default experiments are setup in this repositiory.
Navigate there by
```bash
poetry shell
cd experiments/1_fabric_mpc/pointMass
```

Then the experiment is run with the command line interface
```bash
../../../plannerbenchmark/exec/runner -c setup/exp.yaml -p fabric setup/fabric.yaml --render
```

## Postprocessing

The experiments can be postprocessed using the provide executable.
When using poetry, make sure you are in the virtual environment.
```bash
poetry shell
cd path/to/folder/of/experiment
```
The you can run the post processor with arguments as 
```bash
../../../plannerbenchmark/exec/postProcessor --exp path/to/experiment -k time2Goal pathLength --plot
```

