#!/bin/bash
export PYTHONPATH="$PWD"
pkill -f boardd
nice -3 ~/raspilot/selfdrive/boardd/boardd &
pkill -f controlsd
nice -2 python ~/raspilot/selfdrive/controls/controlsd.py & 
sleep 4
pkill -f laterald
nice -0 python  ~/raspilot/selfdrive/controls/laterald.py &
sleep 2
pkill -f dashboard
nice -2 python ~/raspilot/dashboard.py &
sleep 8
bash ~/raspilot/fix_niceness.sh