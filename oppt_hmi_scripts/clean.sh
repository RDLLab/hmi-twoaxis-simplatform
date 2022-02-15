#!/bin/bash

DIR=../oppt_install/oppt/src/plugins

for dir in $DIR/**
do
    rm -r $dir/HMI*
done