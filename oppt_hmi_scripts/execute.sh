#!/bin/bash

# ./setPipes.sh
RETURN_DIR=`pwd`
cd ../oppt_install/oppt/bin
rm -r log
./abt --cfg ../cfg/HMISolver.cfg
cd $RETURN_DIR
