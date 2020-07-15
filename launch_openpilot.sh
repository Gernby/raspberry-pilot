#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f transcoderd
pkill -f controlsd
pkill -f boardd
pkill -f dashboard
python3.7 selfdrive/controls/transcoderd.py &
python3.7 dashboard.py &
python3.7 selfdrive/controls/controlsd.py & 
python3.7 selfdrive/pandad.py &
#selfdrive/boardd/boardd &

