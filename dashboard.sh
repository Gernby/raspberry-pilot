#!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f dashboard
taskset -a --cpu-list 2,3 python ~/raspilot/dashboard.py &
