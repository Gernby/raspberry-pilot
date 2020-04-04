#!/bin/bash
export PYTHONPATH="$PWD" 
pkill -f transcoderd
nice -1 python3 models/Bosch_GRU_Transcoder.py &
#nice -5 python models/Bosch_GRU_Transcoder.py &
sleep 8
bash ~/raspilot/fix_niceness.sh