#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
python3.7 selfdrive/controls/transcoderd.py &
python3.7 selfdrive/controls/controlsd.py & 
selfdrive/boardd/boardd &
python3.7 dashboard.py &
