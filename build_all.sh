#!/bin/bash

export PYTHONPATH="$PWD"
export PASSIVE="0"

cd ~/raspilot
git clean -xdf
cd selfdrive/boardd
make clean
PYTHONPATH=~/raspilot make 
cd ..
cd can
make clean
PYTHONPATH=~/raspilot make 
cd ..
cd ..
make
