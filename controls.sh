#!/bin/bash
export PYTHONPATH="$PWD"
export PASSIVE="0"
pkill -f boardd
nice -15 ~/raspilot/selfdrive/boardd/boardd &
pkill -f controlsd
nice -1 python ~/raspilot/selfdrive/controls/controlsd.py & 
sleep 2
pkill -f laterald
nice -1 python  ~/raspilot/selfdrive/controls/laterald.py &
sleep 2
pkill -f dashboard
nice -1 python ~/raspilot/dashboard.py &
