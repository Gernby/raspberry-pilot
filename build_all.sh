#!/bin/bash
git clean -xdf
cd selfdrive/boardd
make clean
PYTHONPATH=~/openpilot make 
cd ..
cd can27
make clean
PYTHONPATH=~/openpilot make 
cd ..
cd can
make clean
PYTHONPATH=~/openpilot make 
cd ..
cd ..
make
