#!/bin/bash

export PYTHONPATH="$PWD"
export PASSIVE="0"

cd ~/raspilot
pipenv shell
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
cd panda/board
pkill -f boardd
PYTHONPATH=~/raspilot make clean
PYTHONPATH=~/raspilot make 
