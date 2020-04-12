#!/bin/bash
export PYTHONPATH="$PWD"
pkill -f boardd
nice -4 ~/raspilot/selfdrive/boardd/boardd &
pkill -f controlsd
nice -2 python ~/raspilot/selfdrive/controls/controlsd.py & 
pkill -f dashboard
nice -3 python ~/raspilot/dashboard.py &
sleep 8
bash ~/raspilot/fix_niceness.sh