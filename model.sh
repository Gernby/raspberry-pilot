#!/bin/bash
pkill -f transcoderd
nice -6 python models/Bosch_GRU_Transcoder.py &
