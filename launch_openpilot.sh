#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f transcoderd
pkill -f controlsd
pkill -f pandad
pkill -f boardd
pkill -f ubloxd
pkill -f dashboard
sleep 15
taskset -a --cpu-list 2,3 python3 ~/raspilot/selfdrive/controls/transcoderd.py &
taskset -a --cpu-list 0,1 python3 ~/raspilot/selfdrive/pandad.py &
#taskset -a --cpu-list 0,1 ~/raspilot/selfdrive/boardd/boardd &
taskset -a --cpu-list 0,1 python3 ~/raspilot/selfdrive/controls/controlsd.py &
taskset -a --cpu-list 2,3 ~/raspilot/selfdrive/locationd/ubloxd &
#sleep 15
taskset -a --cpu-list 2,3 python3 ~/raspilot/dashboard.py &
