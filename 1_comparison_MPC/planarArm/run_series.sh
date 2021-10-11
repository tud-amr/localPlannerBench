#!/usr/bin/env bash

if [[ ( "$#" == 1 ) ]]; then
  seriesFolder=$1
else
  seriesFolder="results"
fi
nCases=10
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
