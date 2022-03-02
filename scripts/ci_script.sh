#!/bin/bash

cd experiments/example
../../plannerbenchmark/exec/runner -c setup/exp.yaml -p pdplanner setup/planner.yaml
cd ../..
pytest tests
rm -r experiments/example/results/pdplanner_*

