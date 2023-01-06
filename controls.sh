#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f controlsd
pkill -f pandad
pkill -f boardd
pkill -f ubloxd
pkill -f upload_files.py
taskset -a --cpu-list 0,1 python ~/raspilot/selfdrive/controls/controlsd.py &
taskset -a --cpu-list 0,1 python ~/raspilot/selfdrive/pandad.py &  > /dev/null 2>&1
#taskset -a --cpu-list 0,1 ~/raspilot/selfdrive/boardd/boardd &
taskset -a --cpu-list 2,3 ~/raspilot/selfdrive/locationd/ubloxd &  > /dev/null 2>&1
pkill -f dashboard
taskset -a --cpu-list 2,3 python ~/raspilot/dashboard.py &
sleep 10
bash ~/raspilot/fix_niceness.sh