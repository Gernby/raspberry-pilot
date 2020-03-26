#!/bin/bash
export PYTHONPATH="$PWD"
pkill -f boardd
nice -8 ~/raspilot/selfdrive/boardd/boardd &
pkill -f controlsd
nice -6 python ~/raspilot/selfdrive/controls/controlsd.py & 
sleep 4
pkill -f laterald
nice -4 python  ~/raspilot/selfdrive/controls/laterald.py &
sleep 10
pkill -f dashboard
python ~/raspilot/dashboard.py &
