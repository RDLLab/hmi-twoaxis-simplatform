#!/bin/bash

RETURN_DIR=`pwd`
cd ../oppt_install/oppt/src/build
make && make install
# source ~/oppt_test_install/share/oppt/setup.sh
cd $RETURN_DIR