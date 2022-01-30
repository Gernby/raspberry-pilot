#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f transcoderd
python3 selfdrive/controls/transcoderd.py &
#taskset -a --cpu-list 2,3 python3 selfdrive/controls/transcoderd2.py &
pkill -f controlsd
pkill -f pandad
pkill -f boardd
pkill -f ubloxd
pkill -f dashboard
python3 ~/raspilot/selfdrive/pandad.py &
#~/raspilot/selfdrive/boardd/boardd &
python3 ~/raspilot/selfdrive/controls/controlsd.py & 
~/raspilot/selfdrive/locationd/ubloxd &
#taskset -a --cpu-list 2,3 python ~/raspilot/dashboard.py &
#sleep 15
python3 ~/raspilot/dashboard.py &
