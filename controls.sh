#!/bin/bash
export PYTHONPATH="$PWD"
pkill -f controlsd
python ~/raspilot/selfdrive/controls/controlsd.py & 
pkill -f boardd
~/raspilot/selfdrive/boardd/boardd &
pkill -f dashboard
python ~/raspilot/dashboard.py &
sleep 10
bash ~/raspilot/fix_niceness.sh