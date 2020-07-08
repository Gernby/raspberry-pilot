#!/bin/bash
export PYTHONPATH="$PWD"
pkill -f controlsd
python3.7 ~/raspilot/selfdrive/controls/controlsd.py & 
pkill -f boardd
~/raspilot/selfdrive/boardd/boardd &
pkill -f dashboard
python3.7 ~/raspilot/dashboard.py &
sleep 10
sh ~/raspilot/fix_niceness.sh
