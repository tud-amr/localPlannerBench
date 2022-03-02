#!/bin/bash

cd experiments/example
../../plannerbenchmark/exec/runner -c setup/exp.yaml -p pdplanner setup/planner.yaml
resultsFolder=$(ls results/)
resPath="results/${resultsFolder}"
../../plannerbenchmark/exec/postProcessor --exp $resPath --plot -k solverTime clearance --no-open
cd ../..
pytest tests
rm -r experiments/example/results/pdplanner_*

