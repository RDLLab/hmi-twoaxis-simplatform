#!/bin/bash
currentDir=`pwd`
cd ../oppt_install/oppt/src
if [ -d build ] ; then
    rm -r build
fi
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$currentDir/../oppt_install" ..
make && make install
cd $currentDir