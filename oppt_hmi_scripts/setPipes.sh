#!/bin/bash

RETURN_DIR=`pwd`
cd ../oppt_install/oppt/
mkdir -p pipes
cd pipes

rm -r pipeToGama
rm -r statePipeToSolver
rm -r observationPipeToSolver
mkfifo pipeToGama
mkfifo statePipeToSolver
mkfifo observationPipeToSolver

cd $RETURN_DIR
