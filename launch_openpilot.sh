#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
~/.local/bin/pipenv run nice -3 selfdrive/boardd/boardd &
~/.local/bin/pipenv run nice -2 python selfdrive/controls/controlsd.py & 
~/.local/bin/pipenv run nice -1 python models/Bosch_GRU_Transcoder.py &
sleep 5
~/.local/bin/pipenv run nice -0 python selfdrive/controls/laterald.py &
~/.local/bin/pipenv run nice -10 python dashboard.py &
