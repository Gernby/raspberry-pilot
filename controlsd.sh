#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
#pkill -f controlsd
sleep 5
python ~/raspilot/selfdrive/controls/controlsd.py
sleep 10
bash ~/raspilot/fix_niceness.sh