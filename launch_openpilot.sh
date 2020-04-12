#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
~/.local/bin/pipenv run python selfdrive/controls/transcoderd.py &
~/.local/bin/pipenv run python selfdrive/controls/controlsd.py & 
~/.local/bin/pipenv run selfdrive/boardd/boardd &
~/.local/bin/pipenv run python dashboard.py &
