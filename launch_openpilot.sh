#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f transcoderd
python3.7 selfdrive/controls/transcoderd.py &
pkill -f controlsd
pkill -f pandad
pkill -f boardd
pkill -f ubloxd
pkill -f dashboard
python ~/raspilot/selfdrive/pandad.py &
#~/raspilot/selfdrive/boardd/boardd &
~/raspilot/selfdrive/locationd/ubloxd &
python ~/raspilot/selfdrive/controls/controlsd.py wait & 
python ~/raspilot/dashboard.py &
