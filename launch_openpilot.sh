#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f transcoderd
python3.7 selfdrive/controls/transcoderd.py &
#taskset -a --cpu-list 0,1,2,3 python3.7 selfdrive/controls/transcoderd.py &
pkill -f controlsd
pkill -f pandad
pkill -f boardd
pkill -f ubloxd
pkill -f dashboard
taskset -a --cpu-list 0,1 python ~/raspilot/selfdrive/pandad.py &
#~/raspilot/selfdrive/boardd/boardd &
taskset -a --cpu-list 2,3 ~/raspilot/selfdrive/locationd/ubloxd &
#taskset -a --cpu-list 0,1 python ~/raspilot/selfdrive/controls/controlsd.py wait & 
#taskset -a --cpu-list 2,3 python ~/raspilot/dashboard.py &
