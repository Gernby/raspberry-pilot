#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1
export PASSIVE="0"
pkill -f Bosch_GRU_Transcoder
pkill -f boardd
pkill -f controlsd
pipenv run bash monitor.sh
