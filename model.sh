#!/bin/bash
export PYTHONPATH="$PWD"
pkill -f transcoderd
nice -6 python models/Bosch_GRU_Transcoder.py &
