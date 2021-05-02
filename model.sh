#!/bin/bash
export PYTHONPATH="$PWD" 
pkill -f transcoderd
python3 selfdrive/controls/transcoderd.py &
#nice -5 python models/Bosch_GRU_Transcoder.py &
sleep 8
bash ~/raspilot/fix_niceness.sh