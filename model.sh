#!/bin/bash
export PYTHONPATH="$PWD"
pkill -f transcoderd
nice -6 python3 models/Bosch_GRU_Transcoder.py &
#nice -6 python models/Bosch_GRU_Transcoder.py &
