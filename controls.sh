#!/bin/bash
export PYTHONPATH="$PWD"
export PASSIVE="0"
pkill -f boardd
nice -8 ~/raspilot/selfdrive/boardd/boardd &
pkill -f controlsd
nice -4 python ~/raspilot/selfdrive/controls/controlsd.py & 
sleep 2
pkill -f laterald
nice -2 python  ~/raspilot/selfdrive/controls/laterald.py &
sleep 2
pkill -f dashboard
nice -8 python ~/raspilot/dashboard.py
