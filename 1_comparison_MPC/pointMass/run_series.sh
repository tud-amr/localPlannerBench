#!/usr/bin/env bash

nCases=10
seriesFolder="results"
if [[ ( "$#" == 1 ) ]]; then
  seriesFolder=$1
fi
if [[ ( "$#" == 2 ) ]]; then
  seriesFolder=$1
  nCases=$2
fi
echo "Running series with $nCases in folder $seriesFolder"
nValidCases=0
while (( $nValidCases < $nCases )); do
  errFlag=`./run_experiment.py -case setup/exp.yaml -mpc setup/mpc.yaml -fab setup/fabric.yaml \
    --random-init --random-goal --random-obst --no-verbose --res-folder $seriesFolder`
  if [ -z "$errFlag" ];
  then
    nValidCases=$((nValidCases+1))
  else
    echo $errFlag
  fi
  echo "total number of cases computed $nValidCases"
done
