#!/bin/bash
#cd ~/raspilot
#export PYTHONPATH="$PWD"
pkill -f transcoderd
pkill -f controlsd
pkill -f pandad
pkill -f boardd
pkill -f ubloxd
pkill -f dashboard
pkill -f GernbyMode
python3 ~/raspilot/panda/examples/GernbyMode.py
