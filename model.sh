#!/bin/bash
export PYTHONPATH="$PWD" 
pkill -f transcoderd
python3 selfdrive/controls/transcoderd.py &
bash ~/raspilot/controls.sh &
#sleep 8
#bash ~/raspilot/fix_niceness.sh