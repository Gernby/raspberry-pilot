#!/bin/bash
# This must be run from inside the pipenv!
cd ~/raspilot
cd selfdrive/boardd
make clean
PYTHONPATH=~/raspilot make 
cd ..
cd can
make clean
PYTHONPATH=~/raspilot make 
