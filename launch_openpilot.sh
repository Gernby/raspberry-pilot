!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f transcoderd
pkill -f controlsd
pkill -f pandad
pkill -f boardd
pkill -f ubloxd
pkill -f dashboard
pkill -f GernbyMode
# Try both versions... the one that matches the hardware will win
python3 ~/raspilot/selfdrive/car/tesla/controls_PICAN.py
python3 ~/raspilot/selfdrive/car/tesla/controls_Panda.py
