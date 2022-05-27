#!/bin/bash

cp -v -u ../POMDP/HMISolver.cfg ../oppt_install/oppt/cfg

buildFlag=false

if [ -d "../oppt_install/oppt/src/plugins/HMIShared" ] ; then
    sharedPluginsDir="../oppt_install/oppt/src/plugins/HMIShared"
    for f in ../POMDP/plugins/HMIShared/* ; do
        cp -v -u "$f" "$sharedPluginsDir"
    done
else
    cp -v -r ../POMDP/plugins/HMIShared ../oppt_install/oppt/src/plugins
    buildFlag=true
fi

copyDirs=("HMIHeuristicPlugin" "heuristicPlugins" "HMIInitialBeliefPlugin" "initialBeliefPlugins" "HMIObservationPlugin" "observationPlugins" "HMIRewardPlugin" "rewardPlugins" "HMITerminalPlugin" "terminalPlugins" "HMITransitionPlugin" "transitionPlugins" "HMITransitionExecutionPlugin" "transitionPlugins" "HMIObservationExecutionPlugin" "observationPlugins")
len=${#copyDirs[@]}

for (( i=0; i<$len; i=$i+2 ))
do
    sourceDir="../POMDP/plugins/${copyDirs[$i]}"
    destDir="../oppt_install/oppt/src/plugins/${copyDirs[$i+1]}"
    if [ -d "$destDir/${copyDirs[$i]}" ] ; then
        for f in $sourceDir/*
        do
            cp -v -u "$f" "$destDir/${copyDirs[$i]}"
        done
    else
        cp -v -r "$sourceDir" "$destDir"
        buildFlag=true
    fi
done

cd ..
rm oppt_install.zip
zip -r oppt_install.zip oppt_install
cd oppt_hmi_scripts

# if [ "$buildFlag" = true ] ; then
#     ./build.sh
# else
#     ./remake.sh
# fi

# source ../oppt_install/share/oppt/setup.sh
