#!/bin/bash
#cd ~/openpilot
export PYTHONPATH="$PWD"
#export OMP_NUM_THREADS=1
#export MKL_NUM_THREADS=1
#export NUMEXPR_NUM_THREADS=1
#export OPENBLAS_NUM_THREADS=1
#export VECLIB_MAXIMUM_THREADS=1
export PASSIVE="0"
pkill -f boardd
nice -8 ~/raspilot/selfdrive/boardd/boardd &
pkill -f controlsd
nice -6 python ~/raspilot/selfdrive/controls/controlsd.py & # > ~/raspilot/controlsd.log &
pkill -f dashboard
#nice -10 python ~/raspilot/dashboard.py 1 &
#sleep 15
pkill -f laterald
nice -4 python ~/raspilot/selfdrive/controls/laterald.py &
