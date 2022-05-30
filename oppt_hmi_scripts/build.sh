#!/bin/bash
currentDir=`pwd`
cd ..
if ! [ -d oppt_install ] ; then
    echo 'Error: could not find directory `oppt_install`.'
    echo 'Have you remembered to unzip `oppt_install.zip`?'
else
    cd ../oppt_install/oppt/src
    if [ -d build ] ; then
        rm -r build
    fi
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX="$currentDir/../oppt_install" ..
    make && make install
fi
cd $currentDir