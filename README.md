# Local Motion Planning Benchmark Suite

This repository is meant to allow quick comparison between different local motion planning
algorithms. Running and postprocessing is available and we aim to offer a nice interface
to implement a wrapper to your own motion planner.

## Installation

For postprocessing you have to install gnuplot. If you are running ubuntu, you could
install it via
```bash
sudo apt install gnuplot
```

You first have to download the repository
```bash
git clone git@github.com:maxspahn/localPlannerBench.git
```

There are two options to install the package. You can either use pip as
```bash
pip3 install .
```

If you want to also generate the virtualenvironment and you are familiar with 
(https://python-poetry.org/docs/)[poetry], consider using it as:

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

