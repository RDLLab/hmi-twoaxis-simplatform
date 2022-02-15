#!/bin/bash

DIRECTORY_NAME="oppt_install"
RETURN_DIR=`pwd`

cd ..
mkdir $DIRECTORY_NAME
cd $DIRECTORY_NAME
git clone https://github.com/RDLLab/oppt.git
cd oppt
source /opt/ros/melodic/setup.sh
chmod +x install_dependencies.sh && ./install_dependencies.sh
cd ..
export oppt_DIR=`pwd`"/lib/cmake/oppt"
cd $RETURN_DIR