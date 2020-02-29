#!/bin/bash
# This must be run from inside the pipenv!
cd selfdrive/can
make clean
PYTHONPATH=~/raspilot make 
cd ..
cd boardd
make clean
PYTHONPATH=~/raspilot make 
