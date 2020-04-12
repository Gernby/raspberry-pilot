#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
~/.local/bin/pipenv run nice -3 selfdrive/boardd/boardd &
~/.local/bin/pipenv run nice -1 python selfdrive/controls/controlsd.py & 
~/.local/bin/pipenv run nice -0 python selfdrive/controls/transcoderd.py &
~/.local/bin/pipenv run nice -2 python dashboard.py &
