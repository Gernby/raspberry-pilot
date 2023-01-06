#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f pandad
pkill -f boardd
taskset -a --cpu-list 0,1 python ~/raspilot/selfdrive/pandad.py &
#taskset -a --cpu-list 0,1 ~/raspilot/selfdrive/boardd/boardd &
sleep 10
bash ~/raspilot/fix_niceness.sh