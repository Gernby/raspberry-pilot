#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
~/.local/bin/pipenv run nice -8 selfdrive/boardd/boardd &
~/.local/bin/pipenv run nice -6 python selfdrive/controls/controlsd.py & 
sleep 5
~/.local/bin/pipenv run nice -4 python models/Bosch_GRU_Transcoder.py &
~/.local/bin/pipenv run nice -2 python selfdrive/controls/laterald.py &
~/.local/bin/pipenv run nice -1 python dashboard.py &
